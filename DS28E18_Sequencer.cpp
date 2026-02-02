#include "DS28E18_Sequencer.h"

DS28E18_Sequencer::DS28E18_Sequencer() {
    clear();
}

void DS28E18_Sequencer::clear() {
    index = 0;
    memset(buffer, 0, sizeof(buffer));
}

void DS28E18_Sequencer::addStart() {
    if (index < 512) buffer[index++] = SEQ_CMD_START;
}

void DS28E18_Sequencer::addStop() {
    if (index < 512) buffer[index++] = SEQ_CMD_STOP;
}

void DS28E18_Sequencer::addDelay(uint8_t delayVal) {
    if (index + 2 < 512) {
        buffer[index++] = SEQ_CMD_DELAY;
        buffer[index++] = delayVal;
    }
}

bool DS28E18_Sequencer::addWrite(uint8_t i2cAddr, const uint8_t* data, uint8_t len) {
    // Needs Opcode (1) + LenByte (1) + AddrByte (1) + Data (len)
    if (index + 3 + len > 512) return false;

    buffer[index++] = SEQ_CMD_WRITE;
    buffer[index++] = len + 1;          // Length = Data len + Address byte
    buffer[index++] = (i2cAddr << 1);   // I2C Write Address (LSB 0)

    for (uint8_t i = 0; i < len; i++) {
        buffer[index++] = data[i];
    }
    return true;
}

bool DS28E18_Sequencer::addWriteByte(uint8_t i2cAddr, uint8_t data) {
    return addWrite(i2cAddr, &data, 1);
}

bool DS28E18_Sequencer::addRead(uint8_t i2cAddr, uint16_t len) {
    // Step 1: Write the I2C Read Header (Addr + 1)
    // This switches the direction of the I2C bus
    if (index + 3 > 512) return false;

    buffer[index++] = SEQ_CMD_WRITE;
    buffer[index++] = 1;                // Length is just the address byte
    buffer[index++] = (i2cAddr << 1) | 1; // I2C Read Address (LSB 1)

    // Step 2: Clock in data from slave
    // Needs Opcode (1) + LenByte (1) + DummyBytes (len)
    // We use SEQ_CMD_READ_NACK (D3) for the last block to signal end of read
    if (index + 2 + len > 512) return false;

    buffer[index++] = SEQ_CMD_READ_NACK; 
    buffer[index++] = (uint8_t)len; // Read Length

    // Fill buffer with 0xFF dummies to reserve space in SRAM
    for (uint8_t i = 0; i < len; i++) {
        buffer[index++] = 0xFF;
    }

    return true;
}

uint8_t* DS28E18_Sequencer::getBuffer() {
    return buffer;
}

uint16_t DS28E18_Sequencer::getLength() {
    return index;
}