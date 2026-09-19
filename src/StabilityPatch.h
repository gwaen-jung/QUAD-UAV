#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

constexpr uint8_t kI2CRecoveryMaxClocks = 9;
constexpr uint8_t kI2CStopDelayUs = 5;
constexpr uint32_t kI2CRecoveryClockHz = 100000;
constexpr uint32_t kI2CTimeoutMs = 4;
constexpr uint32_t kIMUFreshnessTimeoutMs = 120;
constexpr uint32_t kRcFreshnessTimeoutMs = 300;
constexpr uint32_t kMotorFreshnessTimeoutMs = 100;
constexpr uint8_t kTelemetryPacketHeader = 0xA5;

extern uint8_t gStabilityI2CSdaPin;
extern uint8_t gStabilityI2CSclPin;
extern uint32_t gStabilityI2CClockHz;
extern volatile uint32_t gLastImuOkMs;
extern volatile uint32_t gLastRcOkMs;
extern uint16_t gLastSafeThrottle;

inline void i2cInit(uint8_t sda, uint8_t scl, uint32_t clockHz = kI2CRecoveryClockHz) {
  gStabilityI2CSdaPin = sda;
  gStabilityI2CSclPin = scl;
  gStabilityI2CClockHz = clockHz;

  Wire.begin(sda, scl, clockHz);
  Wire.setClock(clockHz);
  Wire.setTimeout(kI2CTimeoutMs);
}

inline void i2cBusRecovery(uint8_t sda, uint8_t scl) {
  Wire.end();

  pinMode(scl, OUTPUT);
  pinMode(sda, INPUT_PULLUP);
  digitalWrite(scl, HIGH);

  for (uint8_t i = 0; i < kI2CRecoveryMaxClocks; ++i) {
    if (digitalRead(sda) == HIGH) {
      break;
    }
    digitalWrite(scl, LOW);
    delayMicroseconds(kI2CStopDelayUs);
    digitalWrite(scl, HIGH);
    delayMicroseconds(kI2CStopDelayUs);
  }

  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(kI2CStopDelayUs);
  digitalWrite(scl, HIGH);
  delayMicroseconds(kI2CStopDelayUs);
  digitalWrite(sda, HIGH);
  delayMicroseconds(kI2CStopDelayUs);

  Wire.begin(sda, scl, gStabilityI2CClockHz);
  Wire.setClock(gStabilityI2CClockHz);
  Wire.setTimeout(kI2CTimeoutMs);
}

void imuFeedWatchdog();
bool imuIsFresh();
void rcFeedWatchdog();
bool rcIsFresh();
void motorFeedWatchdog();
bool motorIsFresh();
uint16_t getFailsafeThrottle(uint16_t currentThrottle, bool linkFresh);

struct __attribute__((packed)) TelemetryPacket {
  uint8_t header = kTelemetryPacketHeader;
  float roll;
  float pitch;
  float yaw;
  uint16_t motorOut[4];
  uint16_t loopHz;
  uint8_t imuOk;
};

