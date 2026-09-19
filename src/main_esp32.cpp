#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

#include "config.h"
#include "RF_Protocol.h"
#include "IMU/IMU.h"
#include "Motor/Motor.h"
#include "Packet/Packet.h"
#include "Shared/SharedData.h"
#include "UART/UART.h"
#include "GPS/GPS.h"

// E3: Serial mutex for synchronized debug output
#if defined(ESP32)
SemaphoreHandle_t gSerialMutex = nullptr;
#endif

// Minimal UART bridge payload used between ESP32 and STM32.
// This is the first pass of the architecture split: ESP32 receives ESP-NOW
// and forwards the latest RC to STM32, while STM32 remains the PID authority.

static RC_Command RCCommand_to_RC_Command(const RCCommand &in) {
  RC_Command out{};
  out.throttle = in.throttle;
  out.roll_sp = (int16_t)map(in.roll, -1000, 1000, -350, 350);
  out.pitch_sp = (int16_t)map(in.pitch, -1000, 1000, -350, 350);
  out.yaw_rate_sp = (int16_t)map(in.yaw, -1000, 1000, -350, 350);
  out.armed = (in.aux & RF_AUX_ARMED) ? 1 : 0;
  out.aux = in.aux;
  out.timestamp = millis();
  return out;
}

static uint8_t gControllerMac[6] = {0};
static bool gHaveControllerMac = false;
static bool controllerPeerAdded = false;

static void addControllerPeerIfNeeded() {
  if (controllerPeerAdded || !gHaveControllerMac) return;

  if (esp_now_is_peer_exist(gControllerMac)) {
    controllerPeerAdded = true;
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, gControllerMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    controllerPeerAdded = true;
  }
}

static void sendTelemetryToController() {
  if (!gHaveControllerMac) return;

  TelemetryPacket packet{};
  const IMU_Data imu = GetIMUSnapshot();
  const Motor_Output motors = GetMotorSnapshot();
  const RC_Command rc = GetRCSnapshot();
  const GPS_Data gps = GetGPSSnapshot();

  float yaw = imu.yaw;
  if (yaw < 0.0f) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;

  packet.header = RF_TELEMETRY_HEADER;
  packet.armed = rc.armed;
  packet.roll = static_cast<int16_t>(imu.roll * 10.0f);
  packet.pitch = static_cast<int16_t>(imu.pitch * 10.0f);
  packet.yaw = static_cast<int16_t>(yaw * 10.0f);
  packet.m1 = motors.m1;
  packet.m2 = motors.m2;
  packet.m3 = motors.m3;
  packet.m4 = motors.m4;
  packet.gpsFix = gps.fixValid;
  packet.gpsSatellites = gps.satellites;
  packet.navMode = rc.aux & (RF_AUX_POSHOLD | RF_AUX_RTH | RF_AUX_GEOFENCE);
  packet.navError = 0;
  if (packet.navMode != 0 && (!gps.fixValid || gps.satellites < 5 || gps.hdop > 4.0f)) {
    packet.navError = 1;
  }
  packet.gpsAltitudeM = gps.altitude_m;
  packet.gpsSpeedKmh = gps.speedKmh;
  packet.checksum = computeTelemetryChecksum(packet);

  addControllerPeerIfNeeded();
  if (controllerPeerAdded) {
    esp_now_send(gControllerMac,
                 reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  }
}

static bool mac_equal(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; ++i) if (a[i] != b[i]) return false;
  return true;
}

static void print_mac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void hexdump(const uint8_t *data, int len) {
  for (int i = 0; i < len; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < len) Serial.print(' ');
  }
}

static uint32_t lastRecvMs = 0;
static bool controlActive = false;
static const uint32_t CONTROL_TIMEOUT_MS = 300; // match STM32 RC freshness watchdog
static volatile bool rcForwardPending = false;
static RC_Command pendingRc{};
static bool lastForwardedArmed = false;

static void onEspNowReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len != (int)sizeof(RCCommand)) return;

  const RCCommand *cmd = reinterpret_cast<const RCCommand *>(incomingData);
  if (cmd->header != RF_PACKET_HEADER) return;

  uint8_t cs = computeChecksum(*cmd);
  if (cmd->checksum != cs) {
    Serial.printf("[ESP32] RC checksum FAIL: rx=%02X calc=%02X header=%04X len=%d\n",
                  cmd->checksum, cs, cmd->header, len);
    return;
  }

#ifdef DEBUG_VERBOSE
  Serial.printf("[ESP32] RX RC from controller: throttle=%d roll=%d pitch=%d yaw=%d aux=0x%02X armed=%d\n",
                cmd->throttle, cmd->roll, cmd->pitch, cmd->yaw, cmd->aux,
                (cmd->aux & RF_AUX_ARMED) ? 1 : 0);
#endif

  memcpy(gControllerMac, mac, 6);
  gHaveControllerMac = true;
  if (!controllerPeerAdded) {
    addControllerPeerIfNeeded();
  }

  // record receive time and mark control active
  lastRecvMs = millis();
  if (!controlActive) {
    controlActive = true;
    Serial.println("[DRONE] CONTROL ACTIVE (received from controller)");
  }

  pendingRc = RCCommand_to_RC_Command(*cmd);
  rcForwardPending = true;
#ifdef DEBUG_VERBOSE
  Serial.printf("[ESP32] queued RC for forward: throttle=%d roll_sp=%d pitch_sp=%d yaw_rate_sp=%d armed=%d aux=0x%02X\n",
                pendingRc.throttle, pendingRc.roll_sp, pendingRc.pitch_sp, pendingRc.yaw_rate_sp,
                pendingRc.armed, pendingRc.aux);
#endif
}

// call this periodically in loop to detect timeout
static void controlActivityPoll() {
  if (controlActive) {
    if (millis() - lastRecvMs > CONTROL_TIMEOUT_MS) {
      controlActive = false;
      Serial.println("[DRONE] CONTROL TIMEOUT -> END (no packets)");

      // Safety failsafe: explicitly notify STM32 to stop driving motors.
      RC_Command failsafe{};
      failsafe.throttle = 1000;
      failsafe.roll_sp = 0;
      failsafe.pitch_sp = 0;
      failsafe.yaw_rate_sp = 0;
      failsafe.armed = 0;
      failsafe.timestamp = millis();
      UpdateRC(failsafe);
      Packet_BuildAndSend(failsafe);
      lastForwardedArmed = false;
      Serial.println("[UART] sent failsafe RC to STM32");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  safe_serial_begin(); // E3: Initialize serial mutex for thread-safe printing

  SharedData_Init();
  UART_Init();
  Packet_Init();
  Motor_Init();
  if (xTaskCreate(IMU_task, "IMUTask", 4096, nullptr, 1, &IMU_handle) != pdPASS) {
    SAFE_SERIAL_LOCK();
    Serial.println("[IMU] task creation failed");
    SAFE_SERIAL_UNLOCK();
  }
  GPS_StartTask();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  
  SAFE_SERIAL_LOCK();
  Serial.print("[DRONE] STA MAC: ");
  Serial.println(WiFi.macAddress());
  if (esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
    Serial.println("[ESP_NOW] wifi channel set to 1");
  } else {
    Serial.println("[ESP_NOW] failed to set wifi channel");
  }

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[ESP_NOW] receiver ready");
  } else {
    Serial.println("[ESP_NOW] init failed");
  }

  Serial.println("[DRONE] ESP32 ready");
  SAFE_SERIAL_UNLOCK();
}

void loop() {
  if (rcForwardPending) {
    RC_Command rc = pendingRc;
    rcForwardPending = false;

#ifdef DEBUG_VERBOSE
    Serial.printf("[ESP32] forwarding RC to STM32: thr=%d roll_sp=%d pitch_sp=%d yaw_sp=%d armed=%d aux=0x%02X\n",
                  rc.throttle, rc.roll_sp, rc.pitch_sp, rc.yaw_rate_sp, rc.armed, rc.aux);
    Serial.println("[ESP32] Packet_BuildAndSend(rc) called");
#endif

    UpdateRC(rc);
    Packet_BuildAndSend(rc);
    if (rc.armed && !lastForwardedArmed) {
      CommandPacket zeroPoint{};
      zeroPoint.cmd = 1;
      zeroPoint.timestamp = millis();
      Packet_BuildAndSend(zeroPoint);
      Serial.println("[ESP32] sent zero point packet to STM32 on START");
    }
    lastForwardedArmed = rc.armed != 0;
    static uint32_t lastRcLogMs = 0;
    if (millis() - lastRcLogMs >= 200) {
      lastRcLogMs = millis();
      Serial.printf("[ESP_NOW] RC forwarded throttle=%d armed=%u\n", rc.throttle, rc.armed);
    }
  }
  UART_Update();
  Motor_Update();
  controlActivityPoll();

  static uint32_t lastTelemetryMs = 0;
  if (millis() - lastTelemetryMs >= 100) {
    lastTelemetryMs = millis();
    sendTelemetryToController();
  }

  delay(20);
}