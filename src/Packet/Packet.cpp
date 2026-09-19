#include "Packet/Packet.h"
#include "UART/UART.h"
#include <string.h>

static uint8_t packetSequence = 0;

uint16_t Packet_CalculateCRC(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void Packet_BuildAndSendGeneric(uint8_t type, const void *payload, uint8_t length) {
  uint8_t frame[6 + 256];
  frame[0] = PKT_SOF;
  frame[1] = type;
  frame[2] = packetSequence++;
  frame[3] = length;

  memcpy(&frame[4], payload, length);
  uint16_t crc = Packet_CalculateCRC(&frame[1], 3 + length);
  frame[4 + length] = static_cast<uint8_t>(crc & 0xFF);
  frame[5 + length] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  UART_SendRaw(frame, 6 + length);
}

void Packet_Init() {
  packetSequence = 0;
}

void Packet_BuildAndSend(const IMU_Data &data) {
  Packet_BuildAndSendGeneric(PKT_TYPE_IMU, &data, sizeof(data));
}

void Packet_BuildAndSend(const RC_Command &data) {
  Packet_BuildAndSendGeneric(PKT_TYPE_RC, &data, sizeof(data));
}

void Packet_BuildAndSend(const Motor_Output &data) {
  Packet_BuildAndSendGeneric(PKT_TYPE_MOTOR, &data, sizeof(data));
}

void Packet_BuildAndSend(const CommandPacket &data) {
  Packet_BuildAndSendGeneric(PKT_TYPE_COMMAND, &data, sizeof(data));
}

void Packet_BuildAndSend(const PID_Config &data) {
  Packet_BuildAndSendGeneric(PKT_TYPE_PID, &data, sizeof(data));
}

void Packet_BuildAndSend(const GPS_Data &data) {
  Packet_BuildAndSendGeneric(PKT_TYPE_GPS, &data, sizeof(data));
}
