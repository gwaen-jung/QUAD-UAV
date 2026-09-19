// accelerometer and gyroscope 
// AttitudeData (quaternion)
// task có nhiệm vụ đọc dữ liệu từ IMU (BNO08x) và gửi dữ liệu qua SPI cho STM32F103C8
// AttitudeData (quaternion) là dữ liệu mô tả hướng (orientation) hiện tại của drone trong không gian 3D, được biểu diễn dưới dạng quaternion.
/*
  Using the BNO08x IMU
  Example : Euler Angles
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Hardware Connections:
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT (đang bỏ)
  A5  --> RST (đang bỏ)

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN

  Serial.print it out at 115200 baud to serial monitor.

*/
#include "IMU.h"
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Wire.h>
#include <math.h>
#include "../config.h"
#include "../Packet/Packet.h"
#include "../StabilityPatch.h"

//--------cấu hình phần cứng -----------------------
#define BNO080_INT_PIN -1 // chân ngắt (interrupt) của BNO08x kết nối với chân D2 của ESP32
#define BNO080_RST_PIN -1 // chân reset của BNO08x kết nối với
#define BNO080_ADDR 0x4B // địa chỉ I2C của BNO08x (mặc định là 0x4B)

//--------đối tượng phần cứng dùng nội bộ -----------------------
BNO08x myIMU; // đối tượng IMU BNO08x

//--------định nghĩa biến toàn cục -----------------------
QueueHandle_t xQueueAttitude ; // queue dùng để gửi dữ liệu quaternion từ task IMU_task sang task main
TaskHandle_t IMU_handle ; // handle của task IMU_task

static float gLastGyroX = 0.0f;
static float gLastGyroY = 0.0f;
static float gLastGyroZ = 0.0f;
static float gLastAccelX = 0.0f;
static float gLastAccelY = 0.0f;
static float gLastAccelZ = 0.0f;

//--------hàm nội bộ-------------------------
static float wrapTo360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

static float wrapDeltaDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static void setReport(void) {
    // Game Rotation Vector gives a gravity-aligned yaw signal that is less
    // sensitive to linear acceleration and is a better fit for drone attitude.
    if (myIMU.enableGameRotationVector(2500) == true) {
        Serial.println(F(" enable Game Rotation Vector"));
    } else {
        Serial.println(F(" enable Game Rotation Vector failed"));
    }

    // Calibrated gyro and accelerometer are delivered as separate sensor reports
    // from the quaternion stream. Cache their latest values and attach them to
    // the packet that is already sent at the UART cadence.
    if (myIMU.enableGyro(20000) == true) {
        Serial.println(F(" enable Gyro"));
    } else {
        Serial.println(F(" enable Gyro failed"));
    }

    if (myIMU.enableAccelerometer(20000) == true) {
        Serial.println(F(" enable Accelerometer"));
    } else {
        Serial.println(F(" enable Accelerometer failed"));
    }
}

//--------LOGIC các hàm giao tiếp -----------------------
bool initIMU() {
    // Tạo queue TRƯỚC khi bất kỳ ai (kể cả IMU_task) có thể gửi dữ liệu vào nó.
    // Queue chứa 1 phần tử AttitudeData - dùng cho pattern "overwrite" (luôn giữ giá trị mới nhất).
    if (xQueueAttitude == NULL) {
        xQueueAttitude = xQueueCreate(1, sizeof(AttitudeData));
        if (xQueueAttitude == NULL) {
            Serial.println(F("Khong the tao xQueueAttitude - het RAM?"));
            return false;
        }
    }

    i2cInit(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
    // ESP32 core dùng Wire.setTimeOut() (chữ O hoa) và có cơ chế tự phục hồi
    // I2C bus (toggle SCL) khi timeout. STM32duino core dùng tên hàm khác
    // (setTimeout, chữ o thường) và theo tài liệu KHÔNG đảm bảo tự phục hồi
    // bus giống ESP32 - chỉ đơn thuần thoát hàm sớm hơn khi quá thời gian.
    // Nếu sau này vẫn gặp I2C lock-up thật trên STM32, có thể cần thêm hàm
    // bus-recovery thủ công (toggle SCL bằng GPIO) - chưa cần vội lúc này.
#if defined(ARDUINO_ARCH_STM32)
    Wire.setTimeOut(50);
#else
    Wire.setTimeOut(50);
#endif
    // Wire.setClock(400000);   // tạm bỏ dòng này khi debug
    if (myIMU.begin(BNO080_ADDR, Wire, BNO080_INT_PIN, BNO080_RST_PIN) == false) {
        Serial.println(F("BNO080 not detected..."));
        return false;
    }
    setReport();
    return true;
}
void IMU_task(void *pvParameters) {
    (void)pvParameters;
    // Khong con vTaskDelete khi init that bai: BNO08x khong co chan RST noi day
    // nen sau khi ESP32 reset no hay bi ket, phai thu lai (kem bus recovery)
    // thay vi bo IMU vinh vien den khi reboot.
    while (!initIMU()) {
        Serial.println(F("[IMU] init failed - bus recovery, retry in 1s"));
        i2cBusRecovery(I2C_SDA_PIN, I2C_SCL_PIN);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    AttitudeData data;
    uint32_t lastPrint_ms = 0; // dùng để giới hạn tần suất in debug, tránh ngập terminal
    uint32_t lastUartImuPacketMs = 0;
    for (;;) {
        if (myIMU.wasReset()) {
            Serial.println(F("BNO080 has reset, reconfiguring."));
            // Cho firmware SH2 nội bộ của BNO085 thời gian boot xong trước khi
            // gửi lệnh SHTP mới - gửi quá sớm là nguyên nhân gây treo I2C bus.
            vTaskDelay(pdMS_TO_TICKS(250));
            setReport();
        }
        // đọc dữ liệu IMU
        if (myIMU.getSensorEvent() == true) {
          const uint8_t sensorEventId = myIMU.getSensorEventID();

          if (sensorEventId == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
            gLastGyroX = myIMU.getGyroX() * RAD_TO_DEG;
            gLastGyroY = myIMU.getGyroY() * RAD_TO_DEG;
            gLastGyroZ = myIMU.getGyroZ() * RAD_TO_DEG;
          }

          if (sensorEventId == SENSOR_REPORTID_ACCELEROMETER) {
            gLastAccelX = myIMU.getAccelX();
            gLastAccelY = myIMU.getAccelY();
            gLastAccelZ = myIMU.getAccelZ();
          }

          if (sensorEventId == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
            data.qw = myIMU.getGameQuatReal();
            data.qx = myIMU.getGameQuatI();
            data.qy = myIMU.getGameQuatJ();
            data.qz = myIMU.getGameQuatK();
            data.accuracyDeg = static_cast<float>(myIMU.getQuatAccuracy());
            data.timestamp_us = micros();

            xQueueOverwrite(xQueueAttitude, &data);

            const uint32_t nowMs = millis();
            if (nowMs - lastUartImuPacketMs >= 20) {
                lastUartImuPacketMs = nowMs;

                IMU_Data packet{};
                packet.roll = atan2f(2.0f * (data.qw * data.qx + data.qy * data.qz),
                                     1.0f - 2.0f * (data.qx * data.qx + data.qy * data.qy)) * RAD_TO_DEG;
                packet.pitch = asinf(constrain(2.0f * (data.qw * data.qy - data.qz * data.qx), -1.0f, 1.0f)) * RAD_TO_DEG;
                packet.yaw = atan2f(2.0f * (data.qw * data.qz + data.qx * data.qy),
                                    1.0f - 2.0f * (data.qy * data.qy + data.qz * data.qz)) * RAD_TO_DEG;
                packet.yaw = wrapTo360(packet.yaw);

                packet.gx = gLastGyroX;
                packet.gy = gLastGyroY;
                packet.gz = gLastGyroZ;
                packet.ax = gLastAccelX;
                packet.ay = gLastAccelY;
                packet.az = gLastAccelZ;

                const GPS_Data gps = GetGPSSnapshot();
                const bool gpsHeadingValid = gps.fixValid &&
                                            gps.lastFixMs > 0 &&
                                            (millis() - gps.lastFixMs) < 5000 &&
                                            gps.speedKmh >= 5.0f;

                if (gpsHeadingValid) {
                    const float yawDelta = wrapDeltaDeg(gps.courseDeg - packet.yaw);
                    const float fusionGain = constrain(gps.speedKmh / 30.0f, 0.05f, 0.35f);
                    if (fabsf(yawDelta) > 1.5f) {
                        packet.yaw = wrapTo360(packet.yaw + yawDelta * fusionGain);
                    }
                }

                packet.timestamp = nowMs;
                const uint8_t quatStatus = myIMU.getQuatAccuracy();
                data.accuracyDeg = static_cast<float>(quatStatus);
                packet.accuracyDeg = data.accuracyDeg;

                // Luon gui goi IMU deu dan (moi 20ms) bat ke accuracy, de STM32
                // khong bao gio bi "doi" du lieu va coi IMU la mat tin hieu.
                // Accuracy that (packet.accuracyDeg) van duoc gui kem theo, va
                // Flight.cpp ben STM32 tu quyet dinh co du an toan de chay PID
                // hay khong (xem imuAccurate trong Flight_Update()). Truoc day
                // dieu kien quatStatus >= IMU_ACCURACY_THRESHOLD o day chan luon
                // ca viec GUI goi, khien STM32 khong nhan duoc IMU moi khi BNO08x
                // con o Low/Unreliable (rat hay gap khi drone dung yen), lam
                // imuIsFresh() het han va PID khong bao gio chay.
                UpdateIMU(packet);
                Packet_BuildAndSend(packet);
            }

            uint32_t now_ms = millis();
            if (now_ms - lastPrint_ms >= 200) {
                lastPrint_ms = now_ms;
                const uint8_t quatStatus = myIMU.getQuatAccuracy();
                const bool statusPass = quatStatus >= static_cast<uint8_t>(IMU_ACCURACY_THRESHOLD);
                Serial.printf("qw=%.3f qx=%.3f qy=%.3f qz=%.3f status=%u/%u %s gz=%.2f deg/s ax=%.2f ay=%.2f az=%.2f\n",
                              data.qw, data.qx, data.qy, data.qz, quatStatus,
                              static_cast<unsigned>(IMU_ACCURACY_THRESHOLD),
                              statusPass ? "PASS" : "BLOCKED",
                              gLastGyroZ, gLastAccelX, gLastAccelY, gLastAccelZ);
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}