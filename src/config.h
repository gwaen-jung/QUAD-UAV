#pragma once

#include <Arduino.h>

#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// E3: Serial mutex for synchronized debug output across tasks
extern SemaphoreHandle_t gSerialMutex;

inline void safe_serial_begin() {
  if (gSerialMutex == nullptr) {
    gSerialMutex = xSemaphoreCreateMutex();
  }
}

#define SAFE_SERIAL_LOCK() do { if (gSerialMutex) xSemaphoreTake(gSerialMutex, portMAX_DELAY); } while(0)
#define SAFE_SERIAL_UNLOCK() do { if (gSerialMutex) xSemaphoreGive(gSerialMutex); } while(0)
#endif

#define UART_BAUDRATE 57600

#define IMU_RATE_HZ 200
#define DEBUG_RATE_HZ 10
#define RC_SEND_RATE_HZ 50
#define IMU_ACCURACY_THRESHOLD 2.0f

#define UART_RX_PIN 16
#define UART_TX_PIN 17

#if defined(ARDUINO_ARCH_STM32) || defined(STM32F1)
extern HardwareSerial DebugSerial;
extern HardwareSerial FlightSerial;
#define DEBUG_SERIAL DebugSerial
#else
#define DEBUG_SERIAL Serial
#endif

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define GPS_RX_PIN 25
#define GPS_TX_PIN 26
#define GPS_BAUDRATE 9600
#define BNO_I2C_ADDRESS 0x4B
#define NAV_AUX_ARMED 0x01
#define NAV_AUX_POSHOLD 0x02
#define NAV_AUX_RTH 0x04
#define NAV_AUX_GEOFENCE 0x08

// Trim offset mặc định bằng 0. Zero point sẽ được áp dụng tại runtime từ IMU hiện tại.
#define IMU_ROLL_TRIM_DEG 0.0f
#define IMU_PITCH_TRIM_DEG 0.0f
#define IMU_YAW_TRIM_DEG 0.0f

#pragma pack(push, 1)
typedef struct IMU_Data {
  float roll;
  float pitch;
  float yaw;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float accuracyDeg;
  uint32_t timestamp;
} IMU_Data;

typedef struct RC_Command {
  int16_t throttle;
  int16_t roll_sp;  // tenths of a degree, range +/-350 = +/-35.0 deg
  int16_t pitch_sp; // tenths of a degree, range +/-350 = +/-35.0 deg
  int16_t yaw_rate_sp;
  uint8_t armed;
  uint8_t aux;
  uint32_t timestamp;
} RC_Command;

typedef struct CommandPacket {
  uint8_t cmd;
  uint32_t timestamp;
} CommandPacket;

typedef struct Motor_Output {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
  uint32_t timestamp;
} Motor_Output;

typedef struct GPS_Data {
  double   lat;
  double   lon;
  float    altitude_m;
  float    speedKmh;
  float    courseDeg;
  uint8_t  satellites;
  float    hdop;
  uint8_t  fixValid;
  uint32_t lastFixMs;
} GPS_Data;

typedef struct System_Status {
  uint8_t armed;
  uint8_t imu_ok;
  uint8_t rc_ok;
  uint32_t timestamp;
} System_Status;

typedef struct PID_Config {
  float roll_kp;
  float roll_ki;
  float roll_kd;
  float pitch_kp;
  float pitch_ki;
  float pitch_kd;
  float yaw_kp;
  float yaw_ki;
  float yaw_kd;
} PID_Config;
#pragma pack(pop)

extern IMU_Data imu;
extern RC_Command rcCommand;
extern Motor_Output motorOutput;
extern GPS_Data gpsData;
extern System_Status systemStatus;
extern PID_Config pidConfig;

void InitSharedData();
void SharedData_Init();

IMU_Data GetIMUSnapshot();
void UpdateIMU(const IMU_Data &newValue);

RC_Command GetRCSnapshot();
void UpdateRC(const RC_Command &newValue);

Motor_Output GetMotorSnapshot();
void UpdateMotor(const Motor_Output &newValue);

GPS_Data GetGPSSnapshot();
void UpdateGPS(const GPS_Data &newValue);

bool IsZeroPointApplied();
float GetZeroPointRollOffset();
float GetZeroPointPitchOffset();
float GetZeroPointYawOffset();
void SetZeroPointFromIMU(const IMU_Data &imu);
void ClearZeroPoint();

System_Status GetSystemStatus();
void UpdateSystemStatus(const System_Status &newValue);

PID_Config GetPIDConfig();
void UpdatePIDConfig(const PID_Config &newValue);
void ResetPIDConfig();
