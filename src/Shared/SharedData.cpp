#include "Shared/SharedData.h"

#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#elif defined(ARDUINO_ARCH_STM32) || defined(STM32F1)
#include <STM32FreeRTOS.h>
#endif

static IMU_Data gImu{};
static RC_Command gRc{};
static Motor_Output gMotor{};
static GPS_Data gGps{};
static System_Status gStatus{};
#if defined(ESP32) || defined(ARDUINO_ARCH_STM32) || defined(STM32F1)
static SemaphoreHandle_t gSharedDataMutex = nullptr;

static void lockSharedData() {
  if (gSharedDataMutex != nullptr) {
    xSemaphoreTake(gSharedDataMutex, portMAX_DELAY);
  }
}

static void unlockSharedData() {
  if (gSharedDataMutex != nullptr) {
    xSemaphoreGive(gSharedDataMutex);
  }
}
#else
static void lockSharedData() {}
static void unlockSharedData() {}
#endif

void SharedData_Init() {
#if defined(ESP32) || defined(ARDUINO_ARCH_STM32) || defined(STM32F1)
  if (gSharedDataMutex == nullptr) {
    gSharedDataMutex = xSemaphoreCreateMutex();
  }
#endif
  lockSharedData();
  memset(&gImu, 0, sizeof(gImu));
  memset(&gRc, 0, sizeof(gRc));
  memset(&gMotor, 0, sizeof(gMotor));
  memset(&gGps, 0, sizeof(gGps));
  memset(&gStatus, 0, sizeof(gStatus));
  gRc.throttle = 1000;
  gGps.altitude_m = -1.0f;
  gGps.hdop = 99.9f;
  gGps.fixValid = 0;
  gStatus.armed = 0;
  unlockSharedData();
}

IMU_Data GetIMUSnapshot() {
  lockSharedData();
  IMU_Data value = gImu;
  unlockSharedData();
  return value;
}

void UpdateIMU(const IMU_Data &newValue) {
  lockSharedData();
  gImu = newValue;
  unlockSharedData();
}

RC_Command GetRCSnapshot() {
  lockSharedData();
  RC_Command value = gRc;
  unlockSharedData();
  return value;
}

void UpdateRC(const RC_Command &newValue) {
  lockSharedData();
  gRc = newValue;
  unlockSharedData();
}

Motor_Output GetMotorSnapshot() {
  lockSharedData();
  Motor_Output value = gMotor;
  unlockSharedData();
  return value;
}

void UpdateMotor(const Motor_Output &newValue) {
  lockSharedData();
  gMotor = newValue;
  unlockSharedData();
}

GPS_Data GetGPSSnapshot() {
  lockSharedData();
  GPS_Data value = gGps;
  unlockSharedData();
  return value;
}

void UpdateGPS(const GPS_Data &newValue) {
  lockSharedData();
  gGps = newValue;
  unlockSharedData();
}

System_Status GetSystemStatus() {
  lockSharedData();
  System_Status value = gStatus;
  unlockSharedData();
  return value;
}

void UpdateSystemStatus(const System_Status &newValue) {
  lockSharedData();
  gStatus = newValue;
  unlockSharedData();
}

void InitSharedData() {
  SharedData_Init();
}
