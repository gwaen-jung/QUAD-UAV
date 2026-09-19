#include "config.h"

// Minimal config implementation to provide PID defaults and zero-point state.
static bool gZeroPointApplied = false;
static float gZeroPointRollOffset = 0.0f;
static float gZeroPointPitchOffset = 0.0f;
static float gZeroPointYawOffset = 0.0f;

static PID_Config gPidConfig = {
  0.60f, 0.040f, 0.03f,
  0.61f, 0.041f, 0.03f,
  1.30f, 0.0f, 0.0f
};

PID_Config GetPIDConfig() {
  return gPidConfig;
}

void UpdatePIDConfig(const PID_Config &newValue) {
  gPidConfig = newValue;
}

void ResetPIDConfig() {
  gPidConfig.roll_kp = 0.60f;
  gPidConfig.roll_ki = 0.040f;
  gPidConfig.roll_kd = 0.03f;
  gPidConfig.pitch_kp = 0.61f;
  gPidConfig.pitch_ki = 0.041f;
  gPidConfig.pitch_kd = 0.03f;
  gPidConfig.yaw_kp = 1.30f;
  gPidConfig.yaw_ki = 0.0f;
  gPidConfig.yaw_kd = 0.0f;
}

bool IsZeroPointApplied() {
  return gZeroPointApplied;
}

float GetZeroPointRollOffset() { return gZeroPointRollOffset; }
float GetZeroPointPitchOffset() { return gZeroPointPitchOffset; }
float GetZeroPointYawOffset() { return gZeroPointYawOffset; }

void SetZeroPointFromIMU(const IMU_Data &imu) {
  gZeroPointRollOffset = imu.roll;
  gZeroPointPitchOffset = imu.pitch;
  gZeroPointYawOffset = imu.yaw;
  gZeroPointApplied = true;
}

void ClearZeroPoint() {
  gZeroPointApplied = false;
  gZeroPointRollOffset = 0.0f;
  gZeroPointPitchOffset = 0.0f;
  gZeroPointYawOffset = 0.0f;
}
