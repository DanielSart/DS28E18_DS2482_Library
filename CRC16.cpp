#include "CRC16.h"

uint16_t DS28E18_CRC::crc16_update(uint16_t crc, uint8_t data) {
    // CRC-16 used by DS28E18 and other Maxim 1-Wire devices
    // Polynomial : 0xA001 (reflected form of 0x8005)
    // Bit order  : LSB-first (right-shifting)
    // Init value : 0x0000
    // Final XOR  : 0xFFFF (applied once after all bytes are processed)

    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x0001)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc >>= 1;
    }
    return crc;
}