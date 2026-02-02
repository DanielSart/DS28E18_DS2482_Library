#ifndef DS28E18_SEQUENCER_H
#define DS28E18_SEQUENCER_H

#include <Arduino.h>

// DS28E18 Sequencer Opcodes
#define SEQ_CMD_START       0x02
#define SEQ_CMD_STOP        0x03
#define SEQ_CMD_WRITE       0xE3
#define SEQ_CMD_READ        0xD4
#define SEQ_CMD_READ_NACK   0xD3
#define SEQ_CMD_DELAY       0xDD
#define SEQ_CMD_SENS_VDD_ON 0xCC
#define SEQ_CMD_SENS_VDD_OFF 0xBB

class DS28E18_Sequencer {
public:
    DS28E18_Sequencer();

    void clear();
    
    // Adds an I2C Start condition
    void addStart();
    
    // Adds an I2C Stop condition
    void addStop();

    // Generic delay command (val is based on datasheet calculation, 0=1ms)
    void addDelay(uint8_t delayVal);

    // Constructs the [0xE3] [Len] [Addr+W] [Data...] packet
    bool addWrite(uint8_t i2cAddr, const uint8_t* data, uint8_t len);
    
    // Helper for single byte write
    bool addWriteByte(uint8_t i2cAddr, uint8_t data);

    // Constructs the Read sequence: 
    // 1. Writes I2C Address with Read Bit: [0xE3] [1] [Addr+R]
    // 2. Clocks Data: [0xD3] [Len] [FF...] (Using NACK at end to signal stop)
    bool addRead(uint8_t i2cAddr, uint16_t len);

    // Returns pointer to the raw buffer to send to DS28E18
    uint8_t* getBuffer();
    
    // Returns current length of the buffer
    uint16_t getLength();

private:
    uint8_t buffer[512]; // Max sequencer size
    uint16_t index;
};

#endif