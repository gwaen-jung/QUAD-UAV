/*    
mapping chân cho gps về esp32 
vcc-vcc
gnd-gnd
tx-25
rx-26
*/
#include "GPS/GPS.h"
#include "Packet/Packet.h"
#include "Shared/SharedData.h"

#if defined(ESP32)
#include <Arduino.h>
#include <TinyGPSPlus.h>

#define GPS_RAW_TEST_MODE 0

static HardwareSerial SerialGPS(1);
static TinyGPSPlus gps;

TaskHandle_t GPS_handle = nullptr;

void GPS_Init() {
  SerialGPS.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#ifdef DEBUG_VERBOSE
  Serial.printf("[GPS] UART1 ready (RX=%d TX=%d @ %dbps)\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);
#endif
}

#if GPS_RAW_TEST_MODE
static void GPS_RawTestTask(void *pv) {
  (void)pv;
  SerialGPS.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("=== GPS RAW NMEA TEST ===");
  Serial.printf("[GPS-RAW] RX=%d TX=%d baud=%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);
  Serial.println("Waiting for GPS data...\n");

  for (;;) {
    while (SerialGPS.available() > 0) {
      char c = (char)SerialGPS.read();
      Serial.write(c);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
#endif

void GPS_Update() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    GPS_Data d{};
    d.lat = gps.location.lat();
    d.lon = gps.location.lng();
    d.altitude_m = gps.altitude.isValid() ? (float)gps.altitude.meters() : -1.0f;
    d.speedKmh = gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f;
    d.courseDeg = gps.course.isValid() ? (float)gps.course.deg() : 0.0f;
    d.satellites = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    d.hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.9f;
    d.fixValid = 1;
    d.lastFixMs = millis();
    UpdateGPS(d);
    Packet_BuildAndSend(d);

    static uint32_t lastLogMs = 0;
    if (millis() - lastLogMs >= 1000) {
      lastLogMs = millis();
      Serial.printf("[GPS] lat=%.6f lon=%.6f alt=%.1fm sat=%u hdop=%.1f course=%.1f\n",
                    d.lat, d.lon, d.altitude_m, d.satellites, d.hdop, d.courseDeg);
    }
  }

  static uint32_t lastNoDataWarnMs = 0;
  if (gps.charsProcessed() < 10 && millis() - lastNoDataWarnMs >= 5000) {
    lastNoDataWarnMs = millis();
    Serial.println("[GPS] WARNING: chua nhan duoc du lieu NMEA nao - kiem tra day RX/TX/baudrate.");
  }
}

static void GPS_TaskFn(void *pv) {
  (void)pv;
  GPS_Init();
  for (;;) {
    GPS_Update();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void GPS_StartTask() {
#if GPS_RAW_TEST_MODE
  xTaskCreatePinnedToCore(GPS_RawTestTask, "GPSRawTest", 4096, nullptr, 1, &GPS_handle, 0);
#else
  xTaskCreatePinnedToCore(GPS_TaskFn, "GPSTask", 4096, nullptr, 1, &GPS_handle, 0);
#endif
}

#else
TaskHandle_t GPS_handle = nullptr;
void GPS_Init() {}
void GPS_Update() {}
void GPS_StartTask() {}
#endif
