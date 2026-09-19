#include "Web.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <cstring>
#include <cstdio>

#include "../index_html.h"

static WebServer server(80);
static WebSocketsServer webSocket(81);
static bool wsConnected = false;
static uint32_t lastWsSendMs = 0;
static void (*pidHandler)(const PID_Config &pid) = nullptr;
static void (*commandHandler)(const char *command) = nullptr;
static void (*rcHandler)(const RC_Command &rc) = nullptr;

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send_P(200, "text/html", index_html);
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  (void)num;
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      Serial.println("[WEB] client connected");
      break;

    case WStype_DISCONNECTED:
      wsConnected = false;
      Serial.println("[WEB] client disconnected");
      break;

    case WStype_TEXT: {
      if (length > 0 && length < 128) {
        Serial.printf("[WEB] %.*s\n", (int)length, (char *)payload);
        char message[128];
        memcpy(message, payload, length);
        message[length] = '\0';

        if (commandHandler != nullptr &&
            ((length == 10 && memcmp(payload, "zero_point", 10) == 0) ||
             (length == 7 && memcmp(payload, "$ZERO,1", 7) == 0))) {
          commandHandler(length == 7 ? "$ZERO,1" : "zero_point");
        }

        if (rcHandler != nullptr) {
          RC_Command rc{};
          int throttle, roll, pitch, yaw, armed;
          if (sscanf(message, "$RC,%d,%d,%d,%d,%d", &throttle, &roll, &pitch, &yaw, &armed) == 5) {
            rc.throttle = (int16_t)throttle;
            rc.roll_sp = (int16_t)roll;
            rc.pitch_sp = (int16_t)pitch;
            rc.yaw_rate_sp = (int16_t)yaw;
            rc.armed = (uint8_t)armed;
            rc.timestamp = millis();
            rcHandler(rc);
          }
        }

        if (pidHandler != nullptr) {
          PID_Config pid{};
          int parsed = sscanf(message, "$PID,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                              &pid.roll_kp, &pid.roll_ki, &pid.roll_kd,
                              &pid.pitch_kp, &pid.pitch_ki, &pid.pitch_kd,
                              &pid.yaw_kp, &pid.yaw_ki, &pid.yaw_kd);
          if (parsed == 9) {
            pidHandler(pid);
          }
        }
      }
      break;
    }

    default:
      break;
  }
}

void Web_SetPIDHandler(void (*handler)(const PID_Config &pid)) {
  pidHandler = handler;
}

void Web_SetCommandHandler(void (*handler)(const char *command)) {
  commandHandler = handler;
}

void Web_SetRCHandler(void (*handler)(const RC_Command &rc)) {
  rcHandler = handler;
}

void Web_SetBNOStatus(bool connected) {
  (void)connected;
}

void Web_BroadcastLog(const char *txt) {
  if (txt == nullptr || !wsConnected) {
    return;
  }
  webSocket.broadcastTXT(txt);
}

void Web_Init() {
  const char *ssid = "ESP32-Debug-AP";
  const char *password = "12345678";

  // Dùng WIFI_AP_STA (không phải WIFI_AP thuần) để giữ CẢ 2 interface STA
  // và AP cùng hoạt động. Nếu chỉ dùng WIFI_AP, interface STA sẽ tắt hẳn -
  // và nếu địa chỉ MAC mà controller đang nhắm tới (kDroneMac) là MAC của
  // STA (lấy qua WiFi.macAddress() lúc trước), gói tin ESP-NOW sẽ gửi tới
  // một interface không còn tồn tại -> không bao giờ nhận được ACK, dù
  // mọi cấu hình khác đều đúng. Đây chính là nguyên nhân status=FAIL liên
  // tục dù kênh và checksum đều khớp.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  // Force AP to a fixed channel so ESP-NOW peers can be added with the same channel
  const int apChannel = 1;
  WiFi.softAP(ssid, password, apChannel);
  delay(100);
  WiFi.softAPsetHostname("esp32-gcs");

  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WEB] AP ready: %s at %s\n", ssid, ip.toString().c_str());
  Serial.println("[WEB] Open http://192.168.4.1 in your browser");

  // In ra cả 2 MAC để đối chiếu trực tiếp với kDroneMac đang hard-code bên
  // controller - nếu 1 trong 2 dòng dưới đây không khớp giá trị controller
  // đang gửi tới, đó chính là nguyên nhân.
  Serial.print("[WEB] STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("[WEB] AP  MAC: "); Serial.println(WiFi.softAPmacAddress());

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void Web_Update() {
  server.handleClient();
  webSocket.loop();

  if (!wsConnected) {
    return;
  }

  uint32_t now = millis();
  if (now - lastWsSendMs < 200) {
    return;
  }
  lastWsSendMs = now;

  static uint32_t counter = 0;
  char buf[64];
  snprintf(buf, sizeof(buf), "$RPY,0.00,0.00,0.00,0.000,%lu,0,0,0,0,0,0", (unsigned long)counter++);
  webSocket.broadcastTXT(buf);
}