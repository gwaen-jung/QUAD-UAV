#include "StabilityPatch.h"

uint8_t gStabilityI2CSdaPin = I2C_SDA_PIN;
uint8_t gStabilityI2CSclPin = I2C_SCL_PIN;
uint32_t gStabilityI2CClockHz = kI2CRecoveryClockHz;
volatile uint32_t gLastImuOkMs = 0;
volatile uint32_t gLastRcOkMs = 0;
volatile uint32_t gLastMotorOkMs = 0;
uint16_t gLastSafeThrottle = 1000;

void imuFeedWatchdog() {
  gLastImuOkMs = millis();
}

bool imuIsFresh() {
  return (millis() - gLastImuOkMs) < kIMUFreshnessTimeoutMs;
}

void rcFeedWatchdog() {
  gLastRcOkMs = millis();
}

bool rcIsFresh() {
  return (millis() - gLastRcOkMs) < kRcFreshnessTimeoutMs;
}

void motorFeedWatchdog() {
  gLastMotorOkMs = millis();
}

bool motorIsFresh() {
  return (millis() - gLastMotorOkMs) < kMotorFreshnessTimeoutMs;
}

uint16_t getFailsafeThrottle(uint16_t currentThrottle, bool linkFresh) {
  if (linkFresh) {
    gLastSafeThrottle = constrain(currentThrottle, (uint16_t)1000, (uint16_t)2000);
    return gLastSafeThrottle;
  }

  if (gLastSafeThrottle > 1000u) {
    gLastSafeThrottle = (gLastSafeThrottle > 20u) ? gLastSafeThrottle - 20u : 1000u;
  }
  return gLastSafeThrottle;
}
