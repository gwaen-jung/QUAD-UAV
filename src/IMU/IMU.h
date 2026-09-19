#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include <Wire.h>
// STM32duino core KHÔNG tự include FreeRTOS qua Arduino.h như ESP32 -
// phải include tường minh ở đây, vì QueueHandle_t/TaskHandle_t bên dưới
// cần định nghĩa từ FreeRTOS mới hợp lệ. Chỉ include khi build cho STM32,
// vì file này cũng được compile chung cho ESP32 (ESP32 core đã tự có sẵn
// các kiểu FreeRTOS này qua Arduino.h rồi, không cần include thêm).
#if defined(ARDUINO_ARCH_STM32)
#include <STM32FreeRTOS.h>
#endif

//--------cấu trúc dữ liệu chia sẻ ----------------
typedef struct {
  float qw , qx , qy , qz ; // quaternion
  float accuracyDeg ; // độ chính xác của dữ liệu quaternion (đơn vị degeree)
  uint32_t timestamp_us ; // thời gian lấy dữ liệu quaternion (đơn vị microsecond)
} AttitudeData;

// --------khai báo toàn cục -----------------------
extern QueueHandle_t xQueueAttitude; // 
extern TaskHandle_t IMU_handle;

// --------khai báo hàm giao tiếp -----------------------
void IMU_task(void*pvParameters); // hàm task đọc dữ liệu từ IMU (BNO08x) và gửi dữ liệu qua SPI cho STM32F103C8
bool initIMU(); // hàm khởi tạo IMU (BNO08x)
bool readIMU(AttitudeData *attitude); // hàm đọc dữ liệu quaternion 

#endif