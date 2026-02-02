#ifndef DS28E18_H
#define DS28E18_H

#include <Arduino.h>
#include <Adafruit_DS248x.h>
#include "DS28E18_Sequencer.h"
#include "CRC16.h"

// ---------- Debug Mode Global Switch ----------
extern bool DS28E18_Debug;

// ---------- Debug Helpers ----------
// output one blank line between higher-level commands
static inline void dbgGap() { if (DS28E18_Debug) Serial.println(); }

// unified formatted print for all 1-Wire transactions
static inline void dbgStep(const char *dir, uint8_t val, const char *note = nullptr)
{
    if (!DS28E18_Debug) return;
    Serial.print("1W ");
    Serial.print(dir);
    Serial.print(": 0x");
    Serial.print(val, HEX);
    if (note) {
        Serial.print(" (");
        Serial.print(note);
        Serial.print(")");
    }
    Serial.println();
}

// shorthand single-argument macros
#define DBG_PRINT(x)   if (DS28E18_Debug) { Serial.print(x); }
#define DBG_PRINTLN(x) if (DS28E18_Debug) { Serial.println(x); }
#define DBG_HEX(x)     if (DS28E18_Debug) { Serial.print("0x"); Serial.print((x), HEX); Serial.print(" "); }

// ---------- GPIO mode constants ----------
enum DS28E18_GPIOMode : uint8_t {
    NO_PULLUP     = 0,   // open drain, external pullup
    WEAK_PULLUP   = 1,   // 25 kOhm internal pullup
    STRONG_PULLUP = 2,   // 2.7 kOhm internal pullup
    PUSH_PULL     = 3    // push-pull drive; uses output latch value
};

// GPIO Pin bit positions (Table 32 of DS28E18 datasheet)
enum DS28E18_Pin : uint8_t {
    GPIOA   = 0,   // bit0 -> GPIOA / SS#
    DS_SCL  = 1,   // bit1 -> SCL / SCLK
    GPIOB   = 2,   // bit2 -> GPIOB / MISO
    DS_SDA  = 3    // bit3 -> SDA / MOSI
};

// ------------------------------------------------------------------------
class DS28E18 {
public:
    DS28E18(Adafruit_DS248x &ds248x);

    bool begin(bool enCRC = true);

    // ROM commands
    bool readROM(uint8_t rom[8]);
    bool setROM(const uint8_t rom[8]);

    bool skipROM();           // enable global Skip-ROM mode
    bool matchROM();          // enable global Match-ROM mode

    // Sequencer memory
    bool writeSequencer(uint16_t addr, const uint8_t *data, uint16_t len);
    bool readSequencer(uint16_t addr, uint8_t *out, uint16_t len, uint16_t &outLen);
    bool runSequencer(uint16_t addr, uint16_t len, uint8_t &result);

    // GPIO
    bool initializeGPIO();
    bool resetDeviceStatus();
    bool writeGPIOConfig(const uint8_t *params, uint16_t len);
    bool readGPIOConfig(uint8_t *out, uint16_t &len);
    bool readGPIOBuffer(uint8_t *out, uint16_t &len);
    bool getGPIOInputStates(uint8_t &hi, uint8_t &lo);
    bool pinMode(uint8_t pin, uint8_t mode);
    void pdsMode(uint8_t pin, bool enable);
    bool digitalWrite(uint8_t pin, uint8_t level);
    int  digitalRead(uint8_t pin);

    // CRC control
    inline void enableCRC(bool enCRC) {
        useCRC = enCRC;
        if (useCRC) {
            DBG_PRINTLN("CRC ENABLED");
        }
        else {
            DBG_PRINTLN("CRC DISABLED");
        }
    }
    bool crcEnabled() const { return useCRC; }

    // Overdrive control
    void enableOverdrive() {
        useOverdrive = true;
        ds.overdriveSpeed(useOverdrive);
        DBG_PRINTLN("OVERDRIVE ENABLED");
    }
    void disableOverdrive() {
        useOverdrive = false;
        ds.overdriveSpeed(useOverdrive);
        DBG_PRINTLN("OVERDRIVE DISABLED");
    }
    bool overdriveEnabled() const { return useOverdrive; }

private:
    Adafruit_DS248x &ds;
    bool useCRC;
    bool useOverdrive;
    bool skipMode;  // true -> Skip-ROM addressing
    bool haveROM;
    uint8_t ROM[8];

    // 1-Wire primitives
    bool oneWireReset();
    bool oneWireWriteByte(uint8_t b);
    bool oneWireReadByte(uint8_t &b);

    bool sendCommandStartAndParams(
            const uint8_t *cmdAndParams, uint8_t cmdLen,
            uint8_t expectedResultLen,
            uint8_t *resultBuffer, uint8_t &resultLen,
            uint16_t tOP_ms,
            bool requireSPU);
};

#endif