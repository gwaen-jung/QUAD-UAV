#include "Motor/Motor.h"
#include "Packet/Packet.h"
#include "Shared/SharedData.h"
#include "StabilityPatch.h"
#include "../config.h"

#if defined(ESP32)
#include <Arduino.h>
#include <DShotRMT.h>

// Thứ tự motor phải khớp mixer X-frame trong Flight.cpp:
// M1 -> GPIO27 (front-left), M2 -> GPIO14 (front-right),
// M3 -> GPIO12 (rear-right), M4 -> GPIO13 (rear-left).
static const gpio_num_t kMotorPins[4] = {
  GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13
};
static DShotRMT motor0(kMotorPins[0], RMT_CHANNEL_0);
static DShotRMT motor1(kMotorPins[1], RMT_CHANNEL_1);
static DShotRMT motor2(kMotorPins[2], RMT_CHANNEL_2);
static DShotRMT motor3(kMotorPins[3], RMT_CHANNEL_3);
static DShotRMT* gMotors[4] = {&motor0, &motor1, &motor2, &motor3};
static bool gDShotReady = false;
static bool gEscArmed = false;
static bool gEscArming = false;
static uint32_t gEscArmStartedMs = 0;
static uint32_t gLastDshotRetryLogMs = 0;

// E2: Smooth motor fallback - avoid abrupt source switching
static Motor_Output gLastKnownMotorOutput{};
static uint32_t gMotorSourceLostMs = 0;
static bool gMotorSourceIsPid = true; // true=PID from STM32, false=RC direct fallback
static const uint32_t kMotorSourceFallbackDelayMs = 150; // grace period before switching to fallback


static void DShot_Init() {
  if (gDShotReady) return;

  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    const bool motorOk = gMotors[i]->begin(DSHOT300, false);
    DEBUG_SERIAL.printf("[Motor] DShotRMT motor %d (GPIO%d): %s\n",
                        i, static_cast<int>(kMotorPins[i]), motorOk ? "OK" : "FAIL");
    ok = ok && motorOk;
  }
  if (!ok) {
    DEBUG_SERIAL.println("[Motor] DShot init failed - motors are disabled; retrying.");
    return;
  }

  gDShotReady = true;
}

static bool DShot_Arm() {
  if (!gDShotReady || gEscArmed) return true;
  if (!gEscArming) {
    gEscArming = true;
    gEscArmStartedMs = millis();
    DEBUG_SERIAL.println("[Motor] ARMING ESC (one time, throttle held at 0)...");
  }

  for (int i = 0; i < 4; ++i) gMotors[i]->sendThrottleValue(0);
  if (millis() - gEscArmStartedMs < 3000) return false;

  gEscArming = false;
  gEscArmed = true;
  DEBUG_SERIAL.println("[Motor] ESC ARMED, switching to throttle output.");
  return true;
}

// Đổi 1000-2000 (µs kiểu PWM) sang 48-2047 (DShot throttle)
static uint16_t MapToDShot(uint16_t pwmUs) {
  pwmUs = constrain(pwmUs, 1000, 2000);
  if (pwmUs <= 1000) return 0;
  return map(pwmUs, 1000, 2000, 48, 2047);
}

static Motor_Output MixRcDirect(const RC_Command &rc) {
  Motor_Output output{};
  const int32_t throttle = constrain(static_cast<int32_t>(rc.throttle), 1000, 2000);

  if (rc.armed == 0 || throttle <= 1000) {
    output.m1 = 1000;
    output.m2 = 1000;
    output.m3 = 1000;
    output.m4 = 1000;
    output.timestamp = millis();
    return output;
  }

  const int32_t roll = constrain(static_cast<int32_t>(rc.roll_sp), -350, 350);
  const int32_t pitch = constrain(static_cast<int32_t>(rc.pitch_sp), -350, 350);
  const int32_t yaw = constrain(static_cast<int32_t>(rc.yaw_rate_sp), -350, 350);

  // Symmetric signed X-frame mix: opposite motors gain while the other pair
  // loses the same command, preventing a permanent M4 yaw bias.
  output.m1 = static_cast<uint16_t>(constrain(throttle + pitch + roll - yaw, 1000L, 2000L));
  output.m2 = static_cast<uint16_t>(constrain(throttle + pitch - roll + yaw, 1000L, 2000L));
  output.m3 = static_cast<uint16_t>(constrain(throttle - pitch - roll - yaw, 1000L, 2000L));
  output.m4 = static_cast<uint16_t>(constrain(throttle - pitch + roll + yaw, 1000L, 2000L));
  output.timestamp = millis();
  return output;
}
#endif

void Motor_Init() {
#if defined(ESP32)
  DShot_Init();
#endif
}

void Motor_Update() {
#if defined(ESP32)
  if (!gDShotReady) {
    DShot_Init();
    if (!gDShotReady && millis() - gLastDshotRetryLogMs >= 1000) {
      gLastDshotRetryLogMs = millis();
      DEBUG_SERIAL.println("[Motor] DShot not ready - motors will not spin.");
    }
  }
  if (!gDShotReady) return;

  const RC_Command rc = GetRCSnapshot();
  if (rc.armed == 0) {
    gEscArming = false;
    Motor_Output safeOutput{};
    safeOutput.m1 = 1000;
    safeOutput.m2 = 1000;
    safeOutput.m3 = 1000;
    safeOutput.m4 = 1000;
    safeOutput.timestamp = millis();
    UpdateMotor(safeOutput);
    if (gEscArmed) {
      for (int i = 0; i < 4; ++i) gMotors[i]->sendThrottleValue(0);
      gEscArmed = false;
      DEBUG_SERIAL.println("[Motor] DISARM - da gui DShot 0.");
    }
    return;
  }

  if (!DShot_Arm()) return;

  // E2: Smooth fallback - track motor source and apply grace period before switching
  bool shouldUsePid = motorIsFresh();
  Motor_Output output;
  
  if (shouldUsePid) {
    // PID source is available, use it and cache it for fallback
    output = GetMotorSnapshot();
    gLastKnownMotorOutput = output;
    if (!gMotorSourceIsPid) {
      gMotorSourceIsPid = true;
      static uint32_t lastMotorSwitchLogMs = 0;
      if (millis() - lastMotorSwitchLogMs >= 500) {
        lastMotorSwitchLogMs = millis();
        DEBUG_SERIAL.println("[Motor] source switch: fallback -> PID");
      }
    }
  } else {
    // PID source lost
    if (gMotorSourceIsPid) {
      // First time losing PID, record the time
      gMotorSourceLostMs = millis();
      gMotorSourceIsPid = false;
      DEBUG_SERIAL.println("[Motor] PID source lost, starting fallback grace period...");
    }
    
    uint32_t timeSinceLost = millis() - gMotorSourceLostMs;
    if (timeSinceLost < kMotorSourceFallbackDelayMs) {
      // Within grace period: hold last known output
      output = gLastKnownMotorOutput;
    } else {
      // Grace period expired: switch to RC direct fallback
      output = MixRcDirect(rc);
      static uint32_t fallbackCountWindowStartMs = 0;
      static uint32_t fallbackCountInWindow = 0;
      const uint32_t fallbackNowMs = millis();
      ++fallbackCountInWindow;
      if (fallbackCountWindowStartMs == 0) fallbackCountWindowStartMs = fallbackNowMs;
      if (fallbackNowMs - fallbackCountWindowStartMs >= 1000) {
        DEBUG_SERIAL.printf("[Motor] fallback count in last 1s: %lu\n",
                            static_cast<unsigned long>(fallbackCountInWindow));
        fallbackCountWindowStartMs = fallbackNowMs;
        fallbackCountInWindow = 0;
      }
      static uint32_t lastFallbackSwitchLogMs = 0;
      if (fallbackNowMs - lastFallbackSwitchLogMs >= 1000) {
        lastFallbackSwitchLogMs = fallbackNowMs;
        DEBUG_SERIAL.println("[Motor] source switch: PID -> fallback (grace expired)");
      }
    }
  }
  
  const uint16_t dshot1 = MapToDShot(output.m1);
  const uint16_t dshot2 = MapToDShot(output.m2);
  const uint16_t dshot3 = MapToDShot(output.m3);
  const uint16_t dshot4 = MapToDShot(output.m4);
  gMotors[0]->sendThrottleValue(dshot1);
  gMotors[1]->sendThrottleValue(dshot2);
  gMotors[2]->sendThrottleValue(dshot3);
  gMotors[3]->sendThrottleValue(dshot4);
  // Ghi dung output dang duoc ESP32 phat ra de telemetry gui truc tiep len controller.
  output.timestamp = millis();
  UpdateMotor(output);
  static uint32_t lastMotorLogMs = 0;
  if (millis() - lastMotorLogMs >= 500) {
    lastMotorLogMs = millis();
    const char *sourceStr = gMotorSourceIsPid ? "PID" : "fallback";
    DEBUG_SERIAL.printf("[Motor] source=%s PWM=%u,%u,%u,%u DShot=%u,%u,%u,%u\n",
                        sourceStr,
                        output.m1, output.m2, output.m3, output.m4,
                        dshot1, dshot2, dshot3, dshot4);
  }
#else
  Motor_Output output = GetMotorSnapshot();
  output.timestamp = millis();
  Packet_BuildAndSend(output);
#endif
}
