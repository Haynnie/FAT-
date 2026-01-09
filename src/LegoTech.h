#ifndef LEGOTECH_H
#define LEGOTECH_H

#pragma once
#include <Arduino.h>

// FIRST BYTE
// bits 7-6
#define   MESSAGE_SYS                   0x00    // System message   0b00 << 6
#define   MESSAGE_CMD                   0x40    // Command message  0b01 << 6
#define   MESSAGE_INFO                  0x80    // Info message     0b10 << 6
#define   MESSAGE_DATA                  0xC0    // Data message     0b11 << 6

// bits 5-3
#define   LENGTH_1                      0x00    // 1 byte           0b000 << 3
#define   LENGTH_2                      0x08    // 2 bytes          0b001 << 3
#define   LENGTH_4                      0x10    // 4 bytes          0b010 << 3
#define   LENGTH_8                      0x18    // 8 bytes          0b011 << 3
#define   LENGTH_16                     0x20    // 16 bytes         0b100 << 3
#define   LENGTH_32                     0x28    // 32 bytes         0b101 << 3

// MESSAGE_SYS bits 2-0
#define   BYTE_SYNC                     0x00    // Synchronization byte
#define   BYTE_NACK                     0x02    // Not acknowledge byte (keep alive)
#define   BYTE_ACK                      0x04    // Acknowledge byte

// MESSAGE_CMD bits 2-0
#define   CMD_TYPE                      0x00    // CMD command - TYPE     (device type for VM reference)
#define   CMD_MODES                     0x01    // CMD command - MODES    (number of supported modes minus one)
#define   CMD_SPEED                     0x02    // CMD command - SPEED    (maximum communication speed)
#define   CMD_SELECT                    0x03    // CMD command - SELECT   (select mode)
#define   CMD_WRITE                     0x04    // CMD command - WRITE    (write to device)
#define   CMD_EXT_MODE                  0x06    // CMD command - EXT_MODE (value will be added to mode in CMD_WRITE_DATA - LPF2 only)
#define   CMD_VERSION                   0x07    // CMD command - VERSION  (device firmware and hardware versions)

// MESSAGE_INFO and MESSAGE_DATA bits 2-0
#define   MODE_0                        0x00    // MODE 0 (or 8 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_1                        0x01    // MODE 1 (or 9 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_2                        0x02    // MODE 2 (or 10 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_3                        0x03    // MODE 3 (or 11 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_4                        0x04    // MODE 4 (or 12 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_5                        0x05    // MODE 5 (or 13 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_6                        0x06    // MODE 6 (or 14 if INFO_MODE_PLUS_8 bit is set)
#define   MODE_7                        0x07    // MODE 7 (or 15 if INFO_MODE_PLUS_8 bit is set)

// CMD_EXT_MODE payload
#define   EXT_MODE_0                    0x00    // mode is < 8
#define   EXT_MODE_8                    0x08    // mode is >= 8

// SECOND INFO BYTE
#define   INFO_NAME                     0x00    // INFO command - NAME    (device name)
#define   INFO_RAW                      0x01    // INFO command - RAW     (device RAW value span)
#define   INFO_PCT                      0x02    // INFO command - PCT     (device PCT value span)
#define   INFO_SI                       0x03    // INFO command - SI      (device SI  value span)
#define   INFO_UNITS                    0x04    // INFO command - UNITS   (device SI  unit symbol)
#define   INFO_MAPPING                  0x05    // INFO command - MAPPING (input/output value type flags)
#define   INFO_MODE_COMBOS              0x06    // INFO command - COMBOS  (mode combinations - LPF2-only)
#define   INFO_UNK7                     0x07    // INFO command - unknown (LPF2-only)
#define   INFO_UNK8                     0x08    // INFO command - unknown (LPF2-only)
#define   INFO_UNK9                     0x09    // INFO command - unknown (LPF2-only)
#define   INFO_UNK10                    0x0a    // INFO command - unknown (LPF2-only)
#define   INFO_UNK11                    0x0b    // INFO command - unknown (LPF2-only)
#define   INFO_UNK12                    0x0c    // INFO command - unknown (LPF2-only)
#define   INFO_MODE_PLUS_8              0x20    // Bit flag used in powered up devices to indicate that the mode is 8 + the mode specified in the first byte
#define   INFO_FORMAT                   0x80    // INFO command - FORMAT  (device data sets and format)

// INFO_FORMAT formats
#define   DATA8                         0x00    // 8-bit signed integer
#define   DATA16                        0x01    // 16-bit little-endian signed integer
#define   DATA32                        0x02    // 32-bit little-endian signed integer
#define   DATAF                         0x03    // 32-bit little-endian IEEE 754 floating point

#define   LegoTechnic                   0x2e

class LegoLPF2 {
public:
    enum MotorMode {
        MODE_POWER = 0,
        MODE_SPEED = 1,
        MODE_POS   = 2
    };

    LegoLPF2(HardwareSerial& uart);

    void begin();
    void poll(); 
    void setMode(MotorMode mode);
    void setPower(int8_t pct);
    void setSpeed(int16_t dps); 

private:
    HardwareSerial& _uart;
    uint8_t _mode;

    void sendCmd(uint8_t header, const uint8_t* data, uint8_t len);
    uint8_t checksum(const uint8_t* buf, uint8_t len);
};





LegoLPF2::LegoLPF2(HardwareSerial& uart)
: _uart(uart), _mode(0) {}

void LegoLPF2::begin() {
    _uart.begin(2400);
    delay(100);

    delay(200);

    _uart.write(BYTE_ACK);

    uint8_t speed[4] = { 0x00, 0xC2, 0x01, 0x00 };
    sendCmd(MESSAGE_CMD | LENGTH_4 | CMD_SPEED, speed, 4);

    delay(50);
    _uart.updateBaudRate(115200);
}

void LegoLPF2::poll() {
    _uart.write(BYTE_NACK);
}

void LegoLPF2::setMode(MotorMode mode) {
    _mode = mode;
    uint8_t data[1] = { (uint8_t)mode };
    sendCmd(MESSAGE_CMD | LENGTH_1 | CMD_SELECT, data, 1);
}

void LegoLPF2::setPower(int8_t pct) {
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    uint8_t data[2];
    data[0] = _mode;
    data[1] = (uint8_t)pct;

    sendCmd(MESSAGE_DATA | LENGTH_2 | _mode, data, 2);
}

void LegoLPF2::setSpeed(int16_t dps) {
    uint8_t data[4];
    data[0] = _mode;
    data[1] = dps & 0xFF;
    data[2] = (dps >> 8) & 0xFF;
    data[3] = 0;

    sendCmd(MESSAGE_DATA | LENGTH_4 | _mode, data, 4);
}

void LegoLPF2::sendCmd(uint8_t header, const uint8_t* data, uint8_t len) {
    uint8_t buf[40];
    buf[0] = header;

    for (uint8_t i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }

    buf[len + 1] = checksum(buf, len + 1);

    _uart.write(buf, len + 2);
}

uint8_t LegoLPF2::checksum(const uint8_t* buf, uint8_t len) {
    uint8_t c = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        c ^= buf[i];
    }
    return c;
}


#endif
