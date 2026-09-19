#ifndef RF_RX_h
#define RF_RX_h

// thư viện RF24.h dùng để giao tiếp với module nRF24L01
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "../RF_Protocol.h" // RCCommand, RF_PIPE_ADDRESS, RF_PAYLOAD_SIZE dung chung voi TX
// STM32duino core KHÔNG tự include FreeRTOS qua Arduino.h như ESP32 -
// phải include tường minh ở đây, vì QueueHandle_t/TaskHandle_t bên dưới
// cần định nghĩa từ FreeRTOS mới hợp lệ. Chỉ include khi build cho STM32,
// vì file này cũng được compile chung cho ESP32 (ESP32 core đã tự có sẵn
// các kiểu FreeRTOS này qua Arduino.h rồi, không cần include thêm).
#if defined(ARDUINO_ARCH_STM32)
#include <STM32FreeRTOS.h>
#endif

// --------Cấu trúc dữ liệu chia sẻ ----------------
typedef struct {
  uint8_t data[32]; // dữ liệu nhận được từ tay cầm điều khiển (TX)
  uint32_t timestamp_us; // thời gian nhận được dữ liệu (đơn vị microsecond)
} RFData;

// --------khai báo toàn cục -----------------------
extern QueueHandle_t xQueueRF; // hàng đợi dữ liệu nhận được từ tay cầm điều khiển (TX)

// --------khai báo hàm giao tiếp -----------------------
void RF_task(void*pvParameters); // hàm task nhận dữ liệu từ tay cầm điều khiển (TX) và gửi dữ liệu qua SPI cho STM32F103C8

// --------khai báo hằng số -----------------------
#define RF_CE_PIN  PB0 // chân CE của module nRF24L01
#define RF_CSN_PIN  PA4 // chân CSN của module nRF24L01



#endif