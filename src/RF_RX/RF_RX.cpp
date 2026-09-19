/* nhiệm vụ của lớp RF trong đây sẽ vẫn phối theo bước check SPI của rf sau khi thành công 
sẽ bắt đầu vào bước hứng dữ liệu từ rf của tay cầm điều khiển (TX)
vì vậy RF ở đây sẽ đóng vai trò là bộ nhận dữ liệu (RX)

/*
sơ đồ đấu nối đã chuẩn bị 

 * NRF24L01     Arduino_ Uno  Blue_Pill(stm32f01C)
 * ___________________________________________________
 * VCC  RED    |    3.3v   |      3.3v
 * GND  BROWN  |    GND    |      GND
 * CSN  YELOW  |   Pin10   | A4 NSS1   (PA4)	\
 * CE   ORANGE |   Pin9    | B0 digital(PB0)    |	    NB
 * SCK  GREEN  |   Pin13   | A5 SCK1   (PA5)	|- All these pins
 * MOSI BLUE   |   Pin11   | A7 MOSI1  (PA7)	|  are 3.3v tolerant
 * MISO PURPLE |   Pin12   | A6 MISO1  (PA6) 	/
 *
 *    Always use the adapter plate for 5v!
 */

//--------------gọi tên thư viện-------------------------
#include "RF_RX.h"
#include "../config.h"
#include <RF24.h>
#include <SPI.h>

//--------------khai báo biến toàn cục-------------------------
QueueHandle_t xQueueRF ; // queue dùng để gửi dữ liệu từ task RF_task sang task main

//--------------khai báo biến nội bộ-------------------------
// Đổi tên "audio" -> "radio": đây là module RF24 thu phát vô tuyến, không
// liên quan gì tới âm thanh - tên cũ dễ gây nhầm lẫn, đổi lại cho nhất
// quán với cách đặt tên bên RF_TX.cpp.
static RF24 radio(RF_CE_PIN, RF_CSN_PIN); // tạo đối tượng radio
TaskHandle_t RF_handle; // tạo task handle cho task RF_task

// Địa chỉ pipe và payload size giờ lấy từ RF_Protocol.h (RF_PIPE_ADDRESS,
// RF_PAYLOAD_SIZE) - dùng CHUNG với RF_TX.cpp bên ESP32 controller, không
// còn định nghĩa riêng "RXAA1" / 32 cứng ở đây nữa (đó chính là nguyên
// nhân TX/RX trước đây không nhận được gói tin của nhau).

//--------------hàm nội bộ-------------------------
static bool RF_init() // hàm khởi tạo moudle NRF24L01
{
 if (!radio.begin()) // kiểm tra xem module NRF24L01 có hoạt động không
 {
  DEBUG_SERIAL.println(F("Module NRF24L01 không hoạt động!"));
  return false;
 }
 radio.setChannel(76); // đặt kênh truyền dữ liệu 76
 radio.setPALevel(RF24_PA_LOW); // đặt mức công suất phát thấp
 radio.setDataRate(RF24_1MBPS); // đặt tốc độ truyền dữ liệu 1Mbps
 // Dùng payload cố định RF_PAYLOAD_SIZE byte = sizeof(RCCommand), khớp
 // đúng bên TX - KHÔNG bật enableDynamicPayloads() vì 2 cơ chế loại trừ
 // nhau: dynamic payload để module tự báo độ dài mỗi gói tin (linh hoạt
 // nhưng phức tạp hơn), còn fixed size đơn giản và đủ dùng khi định dạng
 // gói tin đã cố định trước.
 radio.setPayloadSize(RF_PAYLOAD_SIZE);
 radio.openReadingPipe(1, RF_PIPE_ADDRESS); // mở kênh nhận dữ liệu từ tay cầm điều khiển (TX), PHẢI khớp RF_PIPE_ADDRESS bên TX
 radio.setAutoAck(true); // bật chế độ tự động xác nhận dữ liệu nhận được
 // setRetries(delay, count) - đặt tường minh giống hệt bên TX (5, 15) thay
 // vì phụ thuộc giá trị mặc định của thư viện RF24.
 radio.setRetries(5, 15);
 radio.startListening(); // bắt đầu lắng nghe dữ liệu từ tay cầm điều khiển (TX)

 return true;
}

//--------------hàm task nhận dữ liệu từ tay cầm -------------------------
void RF_task(void* pvParameters) // hàm task nhận dữ liệu từ tay cầm điều khiển (TX) và gửi dữ liệu qua SPI cho STM32F103C8
{
 (void) pvParameters; // tránh cảnh báo biến không sử dụng
  // hàm này tạo queue để gửi dữ liệu từ task RF_task sang task main stm32f103c8t6
 if (xQueueRF == NULL) {
     xQueueRF = xQueueCreate(5, sizeof(RFData)); // tạo queue với 5 phần tử, mỗi phần tử có kích thước sizeof(RFData)
 }

 // Giới hạn số lần retry khởi tạo module NRF24L01 để tránh lặp vô hạn chiếm tài nguyên
 const int RF_INIT_MAX_RETRIES = 5;
 int initRetryCount = 0;
  
 while (!RF_init() && initRetryCount < RF_INIT_MAX_RETRIES) {
   DEBUG_SERIAL.printf("[RF] init retry %d/%d\n", initRetryCount + 1, RF_INIT_MAX_RETRIES);
   vTaskDelay(pdMS_TO_TICKS(500));
   initRetryCount++;
 }
  
 // Nếu vẫn không khởi tạo được sau số lần retry, ngừng task
 if (initRetryCount >= RF_INIT_MAX_RETRIES) {
   DEBUG_SERIAL.println(F("[RF] init failed after max retries - RF module not available, suspending task"));
   vTaskSuspend(NULL);
   return;
 }
  
 //-----------------
 RFData rxData;
 for(;;){
  if (radio.available()) {
    // Payload đã cố định RF_PAYLOAD_SIZE byte (xem RF_init), nên đọc thẳng
    // đúng kích thước này, không cần hỏi lại độ dài động từ module.
    radio.read(&rxData.data, RF_PAYLOAD_SIZE);
    rxData.timestamp_us = micros();

    // ---- parse + verify RCCommand trước khi tin dữ liệu ----
    // Trước đây RX chỉ forward byte thô vào queue, không hề biết dữ liệu
    // nhận được có đúng định dạng RCCommand hay không, và checksum bên TX
    // tính ra nhưng chưa từng được kiểm tra lại ở đây - coi như tính năng
    // phát hiện lỗi truyền không có tác dụng gì. Giờ cast lại thành
    // RCCommand rồi kiểm tra header + checksum, chỉ những gói hợp lệ mới
    // được đẩy vào queue cho task xử lý phía sau tin tưởng dùng.
    const RCCommand *cmd = reinterpret_cast<const RCCommand*>(rxData.data);
    bool headerOk   = (cmd->header == RF_PACKET_HEADER);
    bool checksumOk = (cmd->checksum == computeChecksum(*cmd));

    if (headerOk && checksumOk) {
        if (xQueueSend(xQueueRF, &rxData, 0) != pdPASS) {
            RFData Dump;
            xQueueReceive(xQueueRF, &Dump, 0);
            xQueueSend(xQueueRF, &rxData, 0);
        }
    } else {
        // Gói lỗi (nhiễu sóng, mất đồng bộ...) - bỏ qua, KHÔNG đẩy vào
        // queue để task xử lý phía sau (PID/motor) không bao giờ nhận dữ
        // liệu rác.
        DEBUG_SERIAL.print(F("RF: goi tin loi - header="));
        DEBUG_SERIAL.print(headerOk ? F("OK") : F("SAI"));
        DEBUG_SERIAL.print(F(" checksum="));
        DEBUG_SERIAL.println(checksumOk ? F("OK") : F("SAI"));
    }
  }
  vTaskDelay(pdMS_TO_TICKS(2));
 }
}