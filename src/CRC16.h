#ifndef DS28E18_CRC16_H
#define DS28E18_CRC16_H

#include <Arduino.h>

class DS28E18_CRC {
public:
    // Updates an existing CRC16 value with one byte of data.
    static uint16_t crc16_update(uint16_t crc, uint8_t data);
};

#endif