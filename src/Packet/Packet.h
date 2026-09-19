#pragma once

#include "config.h"
#define PKT_SOF 0x7E
#define PKT_TYPE_IMU 0xA1
#define PKT_TYPE_RC 0xA2
#define PKT_TYPE_MOTOR 0xA3
#define PKT_TYPE_COMMAND 0xA4
#define PKT_TYPE_PID 0xA5
#define PKT_TYPE_GPS 0xA6

void Packet_Init();
void Packet_BuildAndSend(const IMU_Data &data);
void Packet_BuildAndSend(const RC_Command &data);
void Packet_BuildAndSend(const Motor_Output &data);
void Packet_BuildAndSend(const CommandPacket &data);
void Packet_BuildAndSend(const PID_Config &data);
void Packet_BuildAndSend(const GPS_Data &data);
uint16_t Packet_CalculateCRC(const uint8_t *data, uint8_t length);
