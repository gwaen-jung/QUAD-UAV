#include "UART/UART.h"
#include "Packet/Packet.h"
#include "Shared/SharedData.h"
#include "StabilityPatch.h"

#include <string.h>

static const uint16_t CRC_POLY = 0xA001;

enum UARTParseState {
  UART_PARSE_SOF,
  UART_PARSE_TYPE,
  UART_PARSE_SEQ,
  UART_PARSE_LENGTH,
  UART_PARSE_PAYLOAD,
  UART_PARSE_CRC_LO,
  UART_PARSE_CRC_HI,
};

static UARTParseState uartState = UART_PARSE_SOF;
static uint8_t uartType = 0;
static uint8_t uartSeq = 0;
static uint8_t uartLength = 0;
static uint8_t uartPayload[256] = {0};
static uint8_t uartPayloadIndex = 0;
static uint16_t uartCrc = 0;
static uint32_t uartAcceptedPackets = 0;
static uint32_t uartCrcErrors = 0;
static uint32_t uartLengthErrors = 0;
static uint32_t uartConsecutiveCrcErrors = 0; // Track burst errors
static uint32_t uartMaxConsecutiveCrcErrors = 0; // Peak value for diagnostics

// Per-packet-type tracking (to see if RC is hit harder than IMU by EMI)
static uint32_t uartImuPackets = 0;
static uint32_t uartRcPackets = 0;
static uint32_t uartGpsPackets = 0;
static uint32_t uartRcCrcErrors = 0;

static void UART_ResetParserState();
static HardwareSerial *uartSerial = nullptr;

#if defined(ESP32)
static SemaphoreHandle_t uartTxMutex = nullptr;
#endif

void UART_Init() {
#ifdef ESP32
  uartSerial = &Serial2;
  uartSerial->begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  if (uartTxMutex == nullptr) {
    uartTxMutex = xSemaphoreCreateMutex();
  }
#else
  uartSerial = &FlightSerial; // USART1: PA9=TX, PA10=RX
  uartSerial->begin(UART_BAUDRATE);
#endif
  delay(100);
  while (uartSerial != nullptr && uartSerial->available()) {
    uartSerial->read();
  }
  DEBUG_SERIAL.println("[UART] serial port ready");
  UART_ResetParserState();
}

void UART_SendRaw(const uint8_t *buffer, uint16_t length) {
  if (!uartSerial || buffer == nullptr || length == 0) return;
#if defined(ESP32)
  if (uartTxMutex != nullptr) {
    xSemaphoreTake(uartTxMutex, portMAX_DELAY);
  }
  uartSerial->write(buffer, length);
  if (uartTxMutex != nullptr) {
    xSemaphoreGive(uartTxMutex);
  }
#else
  uartSerial->write(buffer, length);
#endif
#ifdef DEBUG_UART
  // Also print a hex debug of outgoing frames to debug Serial
  DEBUG_SERIAL.print("[UART->TX] ");
  for (uint16_t i = 0; i < length; ++i) {
    if (buffer[i] < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(buffer[i], HEX);
    DEBUG_SERIAL.print(' ');
  }
  DEBUG_SERIAL.println();
#endif
}

static uint16_t UART_CalculateCRC(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = (crc >> 1) ^ CRC_POLY;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void UART_ResetParserState() {
  uartState = UART_PARSE_SOF;
  uartPayloadIndex = 0;
  uartLength = 0;
  uartCrc = 0;
  memset(uartPayload, 0, sizeof(uartPayload));
}

static void UART_ProcessPacket(uint8_t type, const uint8_t *payload, uint8_t length) {
#ifdef DEBUG_UART
  // Debug: print packet summary and hex payload to debug Serial
  DEBUG_SERIAL.print("[UART->RX] type="); DEBUG_SERIAL.print(type);
  DEBUG_SERIAL.print(" len="); DEBUG_SERIAL.print(length);
  DEBUG_SERIAL.print(" payload:");
  for (uint8_t i = 0; i < length; ++i) {
    if (payload[i] < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(payload[i], HEX);
    DEBUG_SERIAL.print(' ');
  }
  DEBUG_SERIAL.println();
#endif

#ifdef DEBUG_VERBOSE
  DEBUG_SERIAL.printf("[UART] processing type=0x%02X len=%u\n", type, length);
#endif

  if (type == PKT_TYPE_IMU && length != sizeof(IMU_Data)) {
    DEBUG_SERIAL.printf("[UART] drop invalid IMU length=%u expected=%u\n", length, (unsigned)sizeof(IMU_Data));
    return;
  }
  if (type == PKT_TYPE_GPS && length != sizeof(GPS_Data)) {
    DEBUG_SERIAL.printf("[UART] drop invalid GPS length=%u expected=%u\n", length, (unsigned)sizeof(GPS_Data));
    return;
  }
  if (type == PKT_TYPE_RC && length != sizeof(RC_Command)) {
    DEBUG_SERIAL.printf("[UART] drop invalid RC length=%u expected=%u\n", length, (unsigned)sizeof(RC_Command));
    return;
  }
  if (type == PKT_TYPE_COMMAND && length != sizeof(CommandPacket)) {
    DEBUG_SERIAL.printf("[UART] drop invalid CMD length=%u expected=%u\n", length, (unsigned)sizeof(CommandPacket));
    return;
  }

  if (type == PKT_TYPE_IMU && length == sizeof(IMU_Data)) {
    IMU_Data data;
    memcpy(&data, payload, sizeof(data));
#ifdef DEBUG_VERBOSE
    DEBUG_SERIAL.printf("[UART] IMU packet received: roll=%.2f pitch=%.2f yaw=%.2f gz=%.2f acc=%.1f\n",
                       data.roll, data.pitch, data.yaw, data.gz, data.accuracyDeg);
#endif
    UpdateIMU(data);
    imuFeedWatchdog();
  } else if (type == PKT_TYPE_GPS && length == sizeof(GPS_Data)) {
    GPS_Data gps;
    memcpy(&gps, payload, sizeof(gps));
    UpdateGPS(gps);
  } else if (type == PKT_TYPE_RC && length == sizeof(RC_Command)) {
    RC_Command cmd;
    memcpy(&cmd, payload, sizeof(cmd));
    cmd.timestamp = millis();
#ifdef DEBUG_VERBOSE
    DEBUG_SERIAL.printf("[UART] RC packet received: thr=%d roll_sp=%d pitch_sp=%d yaw_rate=%d armed=%d aux=0x%02X\n",
                       cmd.throttle, cmd.roll_sp, cmd.pitch_sp, cmd.yaw_rate_sp,
                       cmd.armed, cmd.aux);
#endif
    UpdateRC(cmd);
    rcFeedWatchdog();
  } else if (type == PKT_TYPE_COMMAND && length == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, payload, sizeof(cmd));
    if (cmd.cmd == 1) {
      SetZeroPointFromIMU(GetIMUSnapshot());
      DEBUG_SERIAL.println("[UART] zero point applied");
    }
  } else if (type == PKT_TYPE_MOTOR && length == sizeof(Motor_Output)) {
    Motor_Output output;
    memcpy(&output, payload, sizeof(output));
    UpdateMotor(output);
    motorFeedWatchdog();
  } else if (type == PKT_TYPE_PID && length == sizeof(PID_Config)) {
    PID_Config config;
    memcpy(&config, payload, sizeof(config));
    UpdatePIDConfig(config);
    DEBUG_SERIAL.println("[UART] PID config applied");
  }

  static uint32_t lastPacketLogMs = 0;
  if (millis() - lastPacketLogMs >= 1000) {
    lastPacketLogMs = millis();
    DEBUG_SERIAL.printf("[UART] packet type=0x%02X len=%u imuFresh=%d rcFresh=%d\n",
              type, length, imuIsFresh(), rcIsFresh());
  }
}

void UART_Update() {
  if (!uartSerial) return;
  while (uartSerial->available()) {
    int raw = uartSerial->read();
    if (raw < 0) {
      break;
    }
 
    uint8_t c = static_cast<uint8_t>(raw);
    if (c == PKT_SOF && uartState != UART_PARSE_SOF) {
      uartState = UART_PARSE_TYPE;
      uartPayloadIndex = 0;
      uartLength = 0;
      uartCrc = 0;
    }

    switch (uartState) {
      case UART_PARSE_SOF:
        if (c == PKT_SOF) {
          uartState = UART_PARSE_TYPE;
        }
        break;

      case UART_PARSE_TYPE:
        uartType = c;
        uartState = UART_PARSE_SEQ;
        break;

      case UART_PARSE_SEQ:
        uartSeq = c;
        uartState = UART_PARSE_LENGTH;
        break;

      case UART_PARSE_LENGTH:
        uartLength = c;
        if (uartLength > sizeof(uartPayload)) {
          uartLengthErrors++;
          uartState = UART_PARSE_SOF;
        } else {
          uartPayloadIndex = 0;
          uartState = UART_PARSE_PAYLOAD;
        }
        break;

      case UART_PARSE_PAYLOAD:
        uartPayload[uartPayloadIndex++] = c;
        if (uartPayloadIndex >= uartLength) {
          uartState = UART_PARSE_CRC_LO;
        }
        break;

      case UART_PARSE_CRC_LO:
        uartCrc = c;
        uartState = UART_PARSE_CRC_HI;
        break;

      case UART_PARSE_CRC_HI: {
        uartCrc |= static_cast<uint16_t>(c) << 8;
        uint8_t frame[3 + 256];
        frame[0] = uartType;
        frame[1] = uartSeq;
        frame[2] = uartLength;
        memcpy(&frame[3], uartPayload, uartLength);
        if (UART_CalculateCRC(frame, 3 + uartLength) == uartCrc) {
          uartAcceptedPackets++;
          uartConsecutiveCrcErrors = 0; // Reset burst counter on success
          // Track per-packet-type
          if (uartType == PKT_TYPE_IMU) uartImuPackets++;
          else if (uartType == PKT_TYPE_RC) uartRcPackets++;
          else if (uartType == PKT_TYPE_GPS) uartGpsPackets++;
          UART_ProcessPacket(uartType, uartPayload, uartLength);
        } else {
          uartCrcErrors++;
          if (uartType == PKT_TYPE_RC) uartRcCrcErrors++;
          uartConsecutiveCrcErrors++;
          if (uartConsecutiveCrcErrors > uartMaxConsecutiveCrcErrors) {
            uartMaxConsecutiveCrcErrors = uartConsecutiveCrcErrors;
          }
        }
        static uint32_t lastParserLogMs = 0;
        if (millis() - lastParserLogMs >= 1000) {
          lastParserLogMs = millis();
          uint32_t totalPackets = uartAcceptedPackets + uartCrcErrors + uartLengthErrors;
          uint32_t errorPercentTimes100 = (totalPackets > 0) ? ((uartCrcErrors + uartLengthErrors) * 10000 / totalPackets) : 0;
          uint32_t errorPct = errorPercentTimes100 / 100;
          uint32_t errorPctFrac = errorPercentTimes100 % 100;
          uint32_t rcTotal = uartRcPackets + uartRcCrcErrors;
          uint32_t rcErrorPct = (rcTotal > 0) ? ((uartRcCrcErrors * 10000) / rcTotal) : 0;
          uint32_t rcErrPct = rcErrorPct / 100;
          uint32_t rcErrPctFrac = rcErrorPct % 100;
          DEBUG_SERIAL.printf("[UART] parser ok=%lu crc_err=%lu len_err=%lu loss_pct=%lu.%02lu burst_max=%lu\n",
                              (unsigned long)uartAcceptedPackets,
                              (unsigned long)uartCrcErrors,
                              (unsigned long)uartLengthErrors,
                              (unsigned long)errorPct,
                              (unsigned long)errorPctFrac,
                              (unsigned long)uartMaxConsecutiveCrcErrors);
          DEBUG_SERIAL.printf("[UART] detail: imu=%lu rc=%lu/%lu(loss=%lu.%02lu%%) gps=%lu\n",
                              (unsigned long)uartImuPackets,
                              (unsigned long)uartRcPackets,
                              (unsigned long)rcTotal,
                              (unsigned long)rcErrPct,
                              (unsigned long)rcErrPctFrac,
                              (unsigned long)uartGpsPackets);
          if (errorPercentTimes100 > 200) { // > 2.00%
            DEBUG_SERIAL.printf("[UART] WARNING: high error rate (%lu.%02lu%%) burst_peak=%lu - check UART wiring/EMI from motors\n", 
                                (unsigned long)errorPct, (unsigned long)errorPctFrac, (unsigned long)uartMaxConsecutiveCrcErrors);
          }
          uartMaxConsecutiveCrcErrors = 0; // Reset peak for next window
        }
        uartState = UART_PARSE_SOF;
        break;
      }
    }
  }
}
