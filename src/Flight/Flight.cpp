#include "Flight/Flight.h"
#include "Shared/SharedData.h"
#include "StabilityPatch.h"
#include "config.h"
#include <math.h>

namespace {
struct PID_State {
  float integral = 0.0f;
  float previousError = 0.0f;
  float filteredDerivative = 0.0f;
};

constexpr float kFallbackPidDt = 1.0f / 500.0f;  // fallback dt
constexpr float kMinValidDt = 0.001f;
constexpr float kMaxValidDt = 0.10f;
constexpr float kPidIntegralLimit = 100.0f;
constexpr float kPidOutputScale = 3.0f;
constexpr int32_t kMixLimit = 250;
constexpr float kPidSaturationLimit = static_cast<float>(kMixLimit) / kPidOutputScale;
constexpr float kDerivativeAlpha = 0.2f;
constexpr float kGpsMaxAgeMs = 2500.0f;
constexpr float kGpsMaxHdop = 4.0f;
constexpr uint8_t kGpsMinSatellites = 5;
constexpr float kPositionGainDegPerMeter = 1.0f;
constexpr float kMaxNavigationTiltDeg = 12.0f;
constexpr float kGeofenceRadiusM = 50.0f;

float UpdatePid(PID_State &state, float error, float kp, float ki, float kd, float dt) {
  const float rawIntegral = state.integral + error * dt;
  const float rawDerivative = (error - state.previousError) / dt;
  state.filteredDerivative += (rawDerivative - state.filteredDerivative) * kDerivativeAlpha;

  const float provisionalOutput = kp * error + ki * rawIntegral + kd * state.filteredDerivative;
  const float saturatedOutput = constrain(provisionalOutput, -kPidSaturationLimit, kPidSaturationLimit);

  if (fabsf(provisionalOutput) <= kPidSaturationLimit) {
    state.integral = constrain(rawIntegral, -kPidIntegralLimit, kPidIntegralLimit);
  }

  state.previousError = error;
  return saturatedOutput;
}
}

static PID_State gRollPid;
static PID_State gPitchPid;
static PID_State gYawPid;

static int32_t gLastRollMix = 0;
static int32_t gLastPitchMix = 0;
static int32_t gLastYawMix = 0;

static uint32_t gLastPidImuTimestamp = 0;
static bool gHasPidImuTimestamp = false;
static GPS_Data gHomeGps{};
static bool gHomeValid = false;
static GPS_Data gHoldGps{};
static bool gHoldValid = false;
static uint8_t gPreviousAux = 0;

static float wrapDegrees(float degrees) {
  while (degrees > 180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

static bool gpsUsable(const GPS_Data &gps) {
  return gps.fixValid != 0 && gps.satellites >= kGpsMinSatellites &&
         gps.hdop <= kGpsMaxHdop && gps.lastFixMs != 0 &&
         millis() - gps.lastFixMs <= static_cast<uint32_t>(kGpsMaxAgeMs);
}

static float distanceBetween(const GPS_Data &from, const GPS_Data &to) {
  const double metersPerDegLat = 111320.0;
  const double metersPerDegLon = 111320.0 * cos(from.lat * DEG_TO_RAD);
  const double northMeters = (to.lat - from.lat) * metersPerDegLat;
  const double eastMeters = (to.lon - from.lon) * metersPerDegLon;
  return sqrtf(static_cast<float>(northMeters * northMeters + eastMeters * eastMeters));
}

static void navigationSetpoints(const GPS_Data &gps, const GPS_Data &target,
                                float yawDeg, float &rollSp, float &pitchSp) {
  const double metersPerDegLat = 111320.0;
  const double metersPerDegLon = 111320.0 * cos(gps.lat * DEG_TO_RAD);
  const double northMeters = (target.lat - gps.lat) * metersPerDegLat;
  const double eastMeters = (target.lon - gps.lon) * metersPerDegLon;
  const float distance = sqrtf(static_cast<float>(northMeters * northMeters + eastMeters * eastMeters));

  if (distance < 2.0f) {
    rollSp = 0.0f;
    pitchSp = 0.0f;
    return;
  }

  const float yawRad = yawDeg * DEG_TO_RAD;
  const float northBody = static_cast<float>(northMeters) * cosf(yawRad) +
                          static_cast<float>(eastMeters) * sinf(yawRad);
  const float eastBody = -static_cast<float>(northMeters) * sinf(yawRad) +
                         static_cast<float>(eastMeters) * cosf(yawRad);
  pitchSp = constrain(northBody * kPositionGainDegPerMeter,
                      -kMaxNavigationTiltDeg, kMaxNavigationTiltDeg);
  rollSp = constrain(eastBody * kPositionGainDegPerMeter,
                     -kMaxNavigationTiltDeg, kMaxNavigationTiltDeg);
}

void Flight_Init() {
  ResetPIDConfig();
  gRollPid = {};
  gPitchPid = {};
  gYawPid = {};
  gLastRollMix = 0;
  gLastPitchMix = 0;
  gLastYawMix = 0;
  gLastPidImuTimestamp = 0;
  gHasPidImuTimestamp = false;
  gHomeGps = {};
  gHomeValid = false;
  gHoldGps = {};
  gHoldValid = false;
  gPreviousAux = 0;
}

void Flight_Update() {
  const PID_Config pidConfig = GetPIDConfig();
  RC_Command rc = GetRCSnapshot();
  IMU_Data imu = GetIMUSnapshot();
  Motor_Output output{};

  const bool imuFresh = imuIsFresh();
  const bool rcFresh = rcIsFresh();
  const GPS_Data gps = GetGPSSnapshot();
  const bool gpsReady = gpsUsable(gps);
  if (rc.armed && !gHomeValid && gpsReady) {
    gHomeGps = gps;
    gHomeValid = true;
    DEBUG_SERIAL.println("[NAV] home captured");
  }
  if (rc.armed == 0) {
    gHomeValid = false;
    gHoldValid = false;
  }

  const bool posHoldRequested = (rc.aux & NAV_AUX_POSHOLD) != 0;
  const bool rthRequested = (rc.aux & NAV_AUX_RTH) != 0;
  const bool geofenceRequested = (rc.aux & NAV_AUX_GEOFENCE) != 0;
  if (posHoldRequested && !gHoldValid && !(gPreviousAux & NAV_AUX_POSHOLD) && gpsReady) {
    gHoldGps = gps;
    gHoldValid = true;
  }
  if (!posHoldRequested) {
    gHoldValid = false;
  }
  gPreviousAux = rc.aux;
  const bool geofenceBreached = geofenceRequested && gHomeValid && gpsReady &&
                                distanceBetween(gps, gHomeGps) > kGeofenceRadiusM;
  const bool navRequested = posHoldRequested || rthRequested || geofenceRequested;
  const bool navActive = navRequested && gHomeValid && gpsReady;
  const bool imuAccurate = isfinite(imu.accuracyDeg) &&
                          imu.accuracyDeg >= IMU_ACCURACY_THRESHOLD;
  if (rc.armed == 0 || rc.throttle <= 1000 || !imuFresh || !imuAccurate || !rcFresh ||
      (navRequested && !navActive)) {
    output.m1 = 1000;
    output.m2 = 1000;
    output.m3 = 1000;
    output.m4 = 1000;
  } else {
    const int32_t rawThrottle = constrain(static_cast<int32_t>(rc.throttle), 1000, 2000);
    const float roll = imu.roll - GetZeroPointRollOffset();
    const float pitch = imu.pitch - GetZeroPointPitchOffset();
    const float yawRate = imu.gz;
    float rollSetpoint = static_cast<float>(rc.roll_sp) / 10.0f;
    float pitchSetpoint = static_cast<float>(rc.pitch_sp) / 10.0f;
    if (navActive) {
      const bool returnToHome = rthRequested || geofenceBreached;
      const GPS_Data &target = returnToHome ? gHomeGps : gHoldGps;
      if (returnToHome || gHoldValid) {
        navigationSetpoints(gps, target, imu.yaw, rollSetpoint, pitchSetpoint);
      }
    }
    const float rollError = rollSetpoint - roll;
    const float pitchError = pitchSetpoint - pitch;
    const float yawError = static_cast<float>(rc.yaw_rate_sp) - yawRate;

    const bool hasNewImuSample = !gHasPidImuTimestamp || imu.timestamp != gLastPidImuTimestamp;
    if (hasNewImuSample) {
      float dt = kFallbackPidDt;
      if (gHasPidImuTimestamp) {
        dt = static_cast<float>(imu.timestamp - gLastPidImuTimestamp) / 1000.0f;
        dt = constrain(dt, kMinValidDt, kMaxValidDt);
      }
      gLastPidImuTimestamp = imu.timestamp;
      gHasPidImuTimestamp = true;
      gLastRollMix = constrain(static_cast<int32_t>(UpdatePid(gRollPid, rollError,
          pidConfig.roll_kp, pidConfig.roll_ki, pidConfig.roll_kd, dt) * kPidOutputScale), -kMixLimit, kMixLimit);
      gLastPitchMix = constrain(static_cast<int32_t>(UpdatePid(gPitchPid, pitchError,
          pidConfig.pitch_kp, pidConfig.pitch_ki, pidConfig.pitch_kd, dt) * kPidOutputScale), -kMixLimit, kMixLimit);
      gLastYawMix = constrain(static_cast<int32_t>(UpdatePid(gYawPid, yawError,
          pidConfig.yaw_kp, pidConfig.yaw_ki, pidConfig.yaw_kd, dt) * kPidOutputScale), -kMixLimit, kMixLimit);
    }

    const int32_t throttle = rawThrottle;
    // X-frame: M1 front-left, M2 front-right, M3 rear-right, M4 rear-left.
    output.m1 = static_cast<uint16_t>(constrain(throttle + gLastPitchMix + gLastRollMix - gLastYawMix, 1000L, 2000L));
    output.m2 = static_cast<uint16_t>(constrain(throttle + gLastPitchMix - gLastRollMix + gLastYawMix, 1000L, 2000L));
    output.m3 = static_cast<uint16_t>(constrain(throttle - gLastPitchMix - gLastRollMix - gLastYawMix, 1000L, 2000L));
    output.m4 = static_cast<uint16_t>(constrain(throttle - gLastPitchMix + gLastRollMix + gLastYawMix, 1000L, 2000L));
  }

  output.timestamp = millis();
  UpdateMotor(output);

  static uint32_t lastLogMs = 0;
  static int lastImuFresh = -1;
  static int lastRcFresh = -1;
  
  // Log state transitions for diagnostic
  if ((imuFresh != lastImuFresh) || (rcFresh != lastRcFresh)) {
    lastImuFresh = imuFresh;
    lastRcFresh = rcFresh;
    DEBUG_SERIAL.printf("[FLT] signal transition: imu=%d rc=%d\n", imuFresh, rcFresh);
  }
  
  if (millis() - lastLogMs > 200) {
    lastLogMs = millis();
    // Khong dung %f: newlib-nano tren STM32 khong link float-printf nen
    // %.1f in ra rong. Chuoi cung duoc rut gon de nam trong 64-byte buffer
    // cua Print::printf(), tranh phai malloc() them tren heap FreeRTOS
    // (heap nho, tung la nghi pham gay tran/crash cung voi stack 384 word cu).
    const int imuStatus = static_cast<int>(imu.accuracyDeg);
    const int threshold = static_cast<int>(IMU_ACCURACY_THRESHOLD);
    DEBUG_SERIAL.printf("[FLT] a=%d thr=%d aux=%02X imu=%d ok=%d q=%d/%d rc=%d nav=%d/%d m=%d,%d,%d,%d\n",
        rc.armed, rc.throttle, rc.aux, imuFresh, imuAccurate,
        imuStatus, threshold, rcFresh,
        navRequested, navActive,
        output.m1, output.m2, output.m3, output.m4);
  }
}