#ifndef RF_PROTOCOL_H
#define RF_PROTOCOL_H

#include <Arduino.h>

// ============================================================================
// RF_Protocol.h - dinh nghia CHUNG cho ca 2 ben TX (esp32_Controler) va
// RX (STM32 drone).
//
// LY DO tach rieng file nay: truoc day RCCommand + dia chi pipe duoc dinh
// nghia doc lap o RF_TX.cpp va RF_RX.cpp -> 2 ben rat de bi lech nhau (dia
// chi pipe khac nhau, payload size khac nhau) ma khong co loi bien dich nao
// bao truoc, chi phat hien duoc khi test thuc te khong nhan duoc goi tin.
// Bang cach include chung 1 file struct/hang so, neu sau nay can doi format
// goi tin chi can sua 1 cho, ca TX va RX tu dong dong bo.
// ============================================================================

// --------dia chi pipe dung chung----------------
// PHAI giong het nhau ca 2 ben: TX openWritingPipe() va RX openReadingPipe()
// cung dung hang so nay.
static const uint8_t RF_PIPE_ADDRESS[6] = "RCC01";

// --------header nhan dang goi tin----------------
#define RF_PACKET_HEADER 0xBC
#define RF_TELEMETRY_HEADER 0xCD
#define RF_AUX_ARMED 0x01
#define RF_AUX_POSHOLD 0x02
#define RF_AUX_RTH 0x04
#define RF_AUX_GEOFENCE 0x08

// --------cau truc goi tin RC----------------
#pragma pack(push, 1)
struct RCCommand {
    uint8_t  header;      // luon la RF_PACKET_HEADER (0xBC)
    int16_t  roll;        // joystick2 VRX
    int16_t  pitch;       // joystick2 VRY
    int16_t  yaw;         // joystick1 VRY
    int16_t  throttle;    // joystick1 VRX
    uint8_t  aux;         // nut bam / switch phu (bitmask) - chua dung toi
    uint8_t  checksum;    // XOR tat ca byte truoc do
};

struct __attribute__((packed)) TelemetryPacket {
    uint8_t  header;      // = RF_TELEMETRY_HEADER
    uint8_t  armed;       // 1 = dang bay that tren drone, 0 = idle
    int16_t  roll;        // do x10 (vd 155 = 15.5 do)
    int16_t  pitch;       // do x10
    int16_t  yaw;         // do x10, 0..3599 (0..359.9 do)
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
    uint8_t  gpsFix;
    uint8_t  gpsSatellites;
    uint8_t  navMode;
    uint8_t  navError;
    float    gpsAltitudeM;
    float    gpsSpeedKmh;
    uint8_t  checksum;    // XOR tat ca byte truoc no
};
#pragma pack(pop)

// Payload size dung chung cho ca radio.setPayloadSize() ben TX lan RX -
// khong duoc hardcode 32 hay bat ky so nao khac nua, luon lay tu sizeof
// struct de tu dong dung khi struct thay doi.
static const uint8_t RF_PAYLOAD_SIZE = sizeof(RCCommand);

// --------ham tinh checksum dung chung----------------
// inline vi header nay duoc include o ca 2 file .cpp khac nhau (TX va RX),
// tranh loi "multiple definition" khi link.
static inline uint8_t computeChecksum(const RCCommand &cmd) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&cmd);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(RCCommand) - 1; i++) {
        sum ^= bytes[i];
    }
    return sum;
}

static inline uint8_t computeTelemetryChecksum(const TelemetryPacket &packet) {
    const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&packet);
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
        sum ^= bytes[i];
    }
    return sum;
}

#endif
