#include "DS28E18.h"
#include <Wire.h>

bool DS28E18_Debug = false;

#define CMD_START               0x66
#define RELEASE_BYTE            0xAA
#define SKIP_ROM                0xCC
#define OVERDRIVE_SKIP_ROM      0x3C
#define MATCH_ROM               0x55
#define OVERDRIVE_MATCH_ROM     0x69
#define RESUME_ROM              0xA5

// ----------------------------------------------
// Constructor
// ----------------------------------------------
DS28E18::DS28E18(Adafruit_DS248x &ds248x) : ds(ds248x) {
    uint8_t arr[8] = {0};
    useCRC   = true;
    haveROM  = false;
    skipMode = false;
    lastROMValid = false;
    memset(ROM, 0, sizeof(arr));
    memset(lastROM, 0, sizeof(lastROM));
}

// ----------------------------------------------
// Begin / basic setup
// ----------------------------------------------
bool DS28E18::begin(bool enCRC) {
    dbgGap();
    DBG_PRINTLN("BEGIN");

    enableCRC(enCRC);
    disableOverdrive();

    if (!ds.reset()) {
        DBG_PRINTLN("DS248x reset FAILED");
        return false;
    }

    ds.activePullup(true);
    haveROM  = false;
    skipMode = false;
    lastROMValid = false;  // Invalidate Resume ROM cache on begin

    DBG_PRINTLN("DS28E18 ready");
    return true;
}

// ----------------------------------------------
// 1-Wire low-level helpers
// ----------------------------------------------
bool DS28E18::oneWireReset() {
    DBG_PRINTLN("1W RESET");
    if (!ds.OneWireReset()) return false;
    ds.busyWait(1000);
    return true;
}

bool DS28E18::oneWireWriteByte(uint8_t b) {
    return ds.OneWireWriteByte(b);
}

bool DS28E18::oneWireReadByte(uint8_t &b) {
    return ds.OneWireReadByte(&b);
}

// ----------------------------------------------
// ROM COMMANDS
// ----------------------------------------------
bool DS28E18::readROM(uint8_t rom[8]) {
    dbgGap();
    DBG_PRINTLN("Read OneWire Device ROM ID");

    uint8_t cmdAndParams[1] = {0x33};

    uint8_t rbuf[8];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmdAndParams,
                                        sizeof(cmdAndParams),
                                        1, rbuf, rlen, 1, false);
    if (!ok) {
        DBG_PRINTLN("Read ROM ID failed");
        return false;
    }

    DBG_PRINT("ROM ID: ")
    for (int i = 0; i < 8; i++){
        DBG_HEX(rbuf[i])
    }
    DBG_PRINTLN();

    DBG_PRINTLN("Read ROM ID success");
    return true;
    /*
    dbgGap();
    DBG_PRINTLN("READ ROM");

    if (!oneWireReset()) return false;
    oneWireWriteByte(0x33);

    for (int i = 0; i < 8; i++)
        oneWireReadByte(rom[i]);

    return true;
    */
}

bool DS28E18::setROM(const uint8_t rom[8]) {
    dbgGap();
    DBG_PRINTLN("SET ROM");

    memcpy(ROM, rom, 8);
    return true;
}

bool DS28E18::skipROM() {
    dbgGap();
    DBG_PRINTLN("SKIP-ROM MODE ENABLED");

    haveROM  = false;
    skipMode = true;
    lastROMValid = false;  // Invalidate Resume ROM cache when switching to skip mode
    return true;
}

bool DS28E18::matchROM() {
    dbgGap();
    DBG_PRINTLN("MATCH-ROM MODE ENABLED");

    haveROM  = true;
    skipMode = false;
    return true;
}

// ----------------------------------------------
// Core command frame
// ----------------------------------------------
bool DS28E18::sendCommandStartAndParams(
    const uint8_t *cmdAndParams,
    uint8_t cmdLen,
    uint8_t expectedResultLen,
    uint8_t *resultBuffer,
    uint8_t &resultLen,
    uint16_t tOP_ms,
    bool requireSPU)
{
    dbgGap();
    DBG_PRINTLN("---- SEND COMMAND START ----");

    if (useOverdrive){
            ds.overdriveSpeed(false);
    }

    if (!oneWireReset()) {
        DBG_PRINTLN("ONEWIRE RESET ERROR");
        return false;
    }

    // ROM addressing
    if (skipMode) {
        if (useOverdrive){
            oneWireWriteByte(OVERDRIVE_SKIP_ROM);
            dbgStep("WRITE", OVERDRIVE_SKIP_ROM, "OVERDRIVE SKIP ROM");
            ds.overdriveSpeed(true);
        } else {
            oneWireWriteByte(SKIP_ROM);
            dbgStep("WRITE", SKIP_ROM, "SKIP ROM");
        }
        lastROMValid = false;  // Skip mode invalidates Resume ROM
    } else if (haveROM) {
        // Check if we can use Resume ROM (same device as last command)
        bool canResume = lastROMValid && (memcmp(ROM, lastROM, 8) == 0);
        
        if (canResume && !useOverdrive) {
            // Use Resume ROM - saves 8 bytes!
            oneWireWriteByte(RESUME_ROM);
            dbgStep("WRITE", RESUME_ROM, "RESUME ROM");
        } else {
            // Full Match ROM with 8-byte address
            if (useOverdrive){
                oneWireWriteByte(OVERDRIVE_MATCH_ROM);
                dbgStep("WRITE", OVERDRIVE_MATCH_ROM, "OVERDRIVE MATCH ROM");
                ds.overdriveSpeed(true);
            } else {
                oneWireWriteByte(MATCH_ROM);
                dbgStep("WRITE", MATCH_ROM, "MATCH ROM");
            }
            for (int i = 0; i < 8; i++) {
                oneWireWriteByte(ROM[i]);
                dbgStep("WRITE", ROM[i], "ROM BYTE");
            }
            // Save this ROM for potential Resume ROM on next command
            memcpy(lastROM, ROM, 8);
            lastROMValid = true;
        }
    } else {
        DBG_PRINTLN("ERROR: no ROM selected and not in skip mode");
        return false;
    }

    // Command Header
    oneWireWriteByte(CMD_START);
    dbgStep("WRITE", CMD_START, "CMD START");

    oneWireWriteByte(cmdLen);
    dbgStep("WRITE", cmdLen, "LENGTH");

    oneWireWriteByte(cmdAndParams[0]);
    dbgStep("WRITE", cmdAndParams[0], "CMD");

    if (cmdLen > 1) {
        DBG_PRINTLN("PAYLOAD:");
        for (uint8_t i = 1; i < cmdLen; i++) {
            oneWireWriteByte(cmdAndParams[i]);
            if (DS28E18_Debug) {
                Serial.print("    0x");
                Serial.println(cmdAndParams[i], HEX);
            }
        }
    }

    // CRC
    uint8_t crcLSB, crcMSB;
    oneWireReadByte(crcLSB); dbgStep("READ", crcLSB, "CRC LSB");
    oneWireReadByte(crcMSB); dbgStep("READ", crcMSB, "CRC MSB");

    // Execute
    oneWireWriteByte(RELEASE_BYTE);
    dbgStep("WRITE", RELEASE_BYTE, "RELEASE");

    if (requireSPU) {
        ds.strongPullup(true);
        DBG_PRINTLN("SPU ON");
    }

    delay(tOP_ms);

    // --- Device reply ---
    DBG_PRINTLN("---- ANSWER START ----");

    uint8_t dummy, rlen;
    oneWireReadByte(dummy); dbgStep("READ", dummy, "DUMMY");
    oneWireReadByte(rlen);  dbgStep("READ", rlen, "LENGTH");

    if (rlen == 0) {
        DBG_PRINTLN("ERROR: zero-length response");
        if (requireSPU) ds.strongPullup(false);
        return false;
    }

    oneWireReadByte(resultBuffer[0]);
    dbgStep("READ", resultBuffer[0], "RESULT");

    if (rlen > 1) {
        DBG_PRINTLN("PAYLOAD:");
        for (uint8_t i = 0; i < rlen - 1; i++) {
            oneWireReadByte(resultBuffer[i + 1]);
            if (DS28E18_Debug) {
                Serial.print("    0x");
                Serial.println(resultBuffer[i + 1], HEX);
            }
        }
    }

    oneWireReadByte(crcLSB); dbgStep("READ", crcLSB, "CRC LSB");
    oneWireReadByte(crcMSB); dbgStep("READ", crcMSB, "CRC MSB");

    if (requireSPU) {
        ds.strongPullup(false);
        DBG_PRINTLN("SPU OFF");
    }

    resultLen = rlen;
    return true;
}

// ----------------------------------------------
// Write Sequencer (11h)
// ----------------------------------------------
bool DS28E18::writeSequencer(uint16_t addr, const uint8_t *data, uint16_t len) {
    if (len == 0 || len > 128) {
        DBG_PRINTLN("writeSequencer: invalid length (1-128 allowed)");
        return false;
    }

    uint8_t cmdAndParams[3 + 128]; // 1 (cmd) + 2 (addr) + up to 128 data
    cmdAndParams[0] = 0x11;        // WRITE SEQUENCER command
    cmdAndParams[1] = addr & 0xFF; // ADDR_LO
    cmdAndParams[2] = (addr >> 8) & 0x01; // only bit0 used as ADDR_HI

    memcpy(&cmdAndParams[3], data, len);

    uint8_t rbuf[8];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmdAndParams,
                                        3 + len,
                                        1, rbuf, rlen,
                                        1 /*tOP*/, false /*no SPU*/);
    if (!ok || rlen == 0) return false;

    if (rbuf[0] == 0xAA) return true;

    DBG_PRINT("writeSequencer failed. Result = 0x");
    if (DS28E18_Debug) Serial.println(rbuf[0], HEX);
    return false;
}

// ----------------------------------------------
// Read Sequencer (22h)
// ----------------------------------------------
bool DS28E18::readSequencer(uint16_t addr, uint8_t *out, uint16_t len, uint16_t &outLen) {
    if (len == 0 || len > 128) {
        DBG_PRINTLN("readSequencer: invalid length (1-128 allowed)");
        return false;
    }

    uint8_t cmd[3];
    cmd[0] = 0x22;        // READ SEQUENCER command
    cmd[1] = addr & 0xFF; // ADDR_LO
    cmd[2] = ((len & 0x7F) << 1) | ((addr >> 8) & 0x01);

    uint8_t tmp[140];
    uint8_t rlen = 0;
    bool ok = sendCommandStartAndParams(cmd, sizeof(cmd), 2, tmp, rlen, 1, false);
    if (!ok || rlen < 2) {
        DBG_PRINTLN("readSequencer: command failed");
        return false;
    }

    // rlen should be 1(result) + nbytes
    uint16_t copyLen = min<uint16_t>(rlen, len);
    memcpy(out, tmp, copyLen);
    outLen = copyLen;

    if (tmp[0] != 0xAA) {
        DBG_PRINT("readSequencer unexpected result: 0x");
        if (DS28E18_Debug) Serial.println(tmp[0], HEX);
        return false;
    }

    return true;
}

// ----------------------------------------------
// Run Sequencer (33h)
// ----------------------------------------------
bool DS28E18::runSequencer(uint16_t addr, uint16_t len, uint8_t &result) {
    if (len == 0 || len > 512) {
        DBG_PRINTLN("runSequencer: invalid len (1-512)");
        return false;
    }

    uint8_t cmd[4];
    cmd[0] = 0x33;        // RUN SEQUENCER command
    cmd[1] = addr & 0xFF; // ADDR_LO
    cmd[2] = ((len & 0x7F) << 1) | ((addr >> 8) & 0x01);
    cmd[3] = (len >> 7) & 0x03; // SLEN_HI bits

    uint8_t rbuf[8];
    uint8_t rlen = 0;
    bool ok = sendCommandStartAndParams(cmd, sizeof(cmd),
                                        1, rbuf, rlen,
                                        5 /*tOP, depends on sequence*/, true /*need SPU*/);
    if (!ok || rlen == 0) {
        DBG_PRINTLN("runSequencer: command failed");
        return false;
    }

    result = rbuf[0];

    if (result == 0xAA) return true;

    DBG_PRINT("runSequencer result code: 0x");
    if (DS28E18_Debug) Serial.println(result, HEX);
    return (result == 0xAA);
}

// ----------------------------------------------
// GPIO INITIALISATION (Table 68)
// ----------------------------------------------
bool DS28E18::initializeGPIO() {
    dbgGap();
    DBG_PRINTLN("POWER-ON GPIO INITIALISATION");

    const uint8_t initParams[4] = {0x0B, 0x03, 0xA5, 0x0F};
    uint8_t cmdAndParams[5] = {0x83, 0x0B, 0x03, 0xA5, 0x0F};

    uint8_t rbuf[8];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmdAndParams,
                                        sizeof(cmdAndParams),
                                        1, rbuf, rlen, 1, false);
    if (!ok) {
        DBG_PRINTLN("GPIO init failed");
        return false;
    }

    DBG_PRINTLN("GPIO initialisation sequence sent");
    return true;
}

// ----------------------------------------------
// GPIO INITIALISATION (Table 68)
// ----------------------------------------------
bool DS28E18::resetDeviceStatus() {
    dbgGap();
    DBG_PRINTLN("Reset Device Status");

    uint8_t cmdAndParams[1] = {0x7A};

    uint8_t rbuf[8];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmdAndParams,
                                        sizeof(cmdAndParams),
                                        1, rbuf, rlen, 1, false);
    if (!ok) {
        DBG_PRINTLN("Reset Device Status failed");
        return false;
    }

    DBG_PRINTLN("Reset Device Status success");
    return true;
}

// ----------------------------------------------
// GPIO COMMANDS
// ----------------------------------------------
bool DS28E18::readGPIOConfig(uint8_t *out, uint16_t &len) {
    dbgGap();
    DBG_PRINTLN("READ GPIO CONFIG");

    uint8_t cmd[3] = {0x7C, 0x0B, 0x03};
    uint8_t tmp[32];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmd, sizeof(cmd), 8, tmp, rlen, 1, false);
    if (!ok) {
        DBG_PRINTLN("READ GPIO CONFIG: frame or CRC error");
        return false;
    }

    if (rlen == 0) {
        DBG_PRINTLN("READ GPIO CONFIG: empty response");
        return false;
    }

    if (tmp[0] == 0x77) {
        DBG_PRINTLN("READ GPIO CONFIG: device replied 0x77 (Invalid input/parameter)");
        return false;
    }

    uint16_t n = min((uint16_t)rlen, len);
    memcpy(out, tmp, n);
    len = n;

    DBG_PRINTLN("READ GPIO CONFIG OK");
    return true;
}

bool DS28E18::writeGPIOConfig(const uint8_t *params, uint16_t len) {
    dbgGap();
    DBG_PRINTLN("WRITE GPIO CONFIG");

    if (len != 2) {
        DBG_PRINTLN("WRITE GPIO CONFIG: expected 2 bytes (HI/LO)");
        return false;
    }

    uint8_t cmdAndParams[5] = {0x83, 0x0B, 0x03, params[0], params[1]};
    uint8_t rbuf[8];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmdAndParams, sizeof(cmdAndParams),
                                        1, rbuf, rlen, 1, false);
    if (!ok) {
        DBG_PRINTLN("WRITE GPIO CONFIG: frame/CRC error");
        return false;
    }

    if (rlen == 0) {
        DBG_PRINTLN("WRITE GPIO CONFIG: empty response");
        return false;
    }

    if (rbuf[0] == 0xAA) {
        DBG_PRINTLN("WRITE GPIO CONFIG OK");
        return true;
    }

    if (rbuf[0] == 0x77) {
        DBG_PRINTLN("WRITE GPIO CONFIG: device replied 0x77 (Invalid input/parameter)");
        DBG_PRINTLN(" -> Verify CFG_REG_TARGET=0x0B, CFG_REG_MOD=0x03, and data bytes.");
    } else {
        DBG_PRINT("Unexpected GPIO result byte: 0x");
        if (DS28E18_Debug)
            Serial.println(rbuf[0], HEX);
    }

    return false;
}

bool DS28E18::readGPIOBuffer(uint8_t *out, uint16_t &len) {
    dbgGap();
    DBG_PRINTLN("READ GPIO BUFFER (INPUT STATE)");

    const uint8_t cmd[3] = {0x7C, 0x0C, 0x03};  // READ GPIO CONFIG, target=0x0C, module=0x03
    uint8_t tmp[8];
    uint8_t rlen = 0;

    bool ok = sendCommandStartAndParams(cmd, sizeof(cmd),
                                        3, tmp, rlen,
                                        1 /*tOP*/, false /*no SPU*/);
    if (!ok) {
        DBG_PRINTLN("READ GPIO BUFFER: frame or CRC error");
        return false;
    }

    if (rlen < 3) {
        DBG_PRINTLN("READ GPIO BUFFER: invalid response length");
        return false;
    }

    // tmp[0] = Result (AAh expected)
    // tmp[1] = GPIO_BUF_HI
    // tmp[2] = GPIO_BUF_LO
    uint16_t n = min<uint16_t>(rlen, len);
    memcpy(out, tmp, n);
    len = n;
    DBG_PRINTLN("READ GPIO BUFFER OK");
    return true;
}

bool DS28E18::getGPIOInputStates(uint8_t &hi, uint8_t &lo)
{
    uint8_t buf[8];
    uint16_t len = sizeof(buf);
    if (!readGPIOBuffer(buf, len))
        return false;
    hi = buf[1];
    lo = buf[2];
    return true;
}

bool DS28E18::pinMode(uint8_t pin, uint8_t mode)
{
    dbgGap();
    DBG_PRINTLN("PINMODE");

    if (pin > 3) {
        DBG_PRINTLN("pinMode: invalid pin index");
        return false;
    }

    uint8_t buf[8];
    uint16_t len = sizeof(buf);
    if (!readGPIOConfig(buf, len)) {
        DBG_PRINTLN("pinMode: readGPIOConfig failed");
        return false;
    }

    // buf[0] = GPIO_CTRL_HI (PW/PS bits)
    // buf[1] = GPIO_CTRL_LO (PDS bits + latch)
    uint8_t hi = buf[1];
    uint8_t lo = buf[2];
    uint8_t mask = (1 << pin);

    // clear old pull-up bits for this pin
    hi &= ~(mask);        // clear PW[n]
    hi &= ~(mask << 4);   // clear PS[n]

    // set new pull-up configuration
    switch (mode) {
        case NO_PULLUP:
            break;                  // leave all bits cleared
        case WEAK_PULLUP:
            hi |= mask;             // PW bit
            break;
        case STRONG_PULLUP:
            hi |= (mask << 4);      // PS bit
            break;
        default:
            DBG_PRINTLN("pinMode: unknown pull-up mode");
            return false;
    }

    // do NOT touch PDS bits here
    uint8_t ctrl[2] = {hi, lo};
    return writeGPIOConfig(ctrl, 2);
}

void DS28E18::pdsMode(uint8_t pin, bool enable)
{
    dbgGap();
    DBG_PRINTLN("PDSMODE");

    if (pin > 3) {
        DBG_PRINTLN("pdsMode: invalid pin index");
        return;
    }

    uint8_t buf[8];
    uint16_t len = sizeof(buf);
    if (!readGPIOConfig(buf, len)) {
        DBG_PRINTLN("pdsMode: readGPIOConfig failed");
        return;
    }

    uint8_t hi = buf[1];
    uint8_t lo = buf[2];
    uint8_t mask = (1 << pin);

    if (enable){
        lo |= mask;     // enable push-pull driver
    } else {
        lo &= ~mask;    // disable (open-drain)
    }

    uint8_t ctrl[2] = {hi, lo};
    if (writeGPIOConfig(ctrl, 2)){
        DBG_PRINTLN("pdsMode: updated successfully");
    } else {
        DBG_PRINTLN("pdsMode: write failed");
    }
}

bool DS28E18::digitalWrite(uint8_t pin, uint8_t level)
{
    dbgGap();
    DBG_PRINTLN("DIGITALWRITE");

    if (pin > 3) return false;

    uint8_t buf[8];
    uint16_t len = sizeof(buf);

    // read current configuration
    if (!readGPIOConfig(buf, len)) return false;

    uint8_t hi   = buf[1];   // pull-up bits
    uint8_t lo   = buf[2];   // latch + PDS bits
    uint8_t mask = (1 << pin);

    // Set or clear the logical output latch
    if (level == HIGH)
        lo |= mask;
    else
        lo &= ~mask;

    uint8_t ctrl[2] = {hi, lo};
    return writeGPIOConfig(ctrl, 2);
}

int DS28E18::digitalRead(uint8_t pin)
{
    dbgGap();
    DBG_PRINTLN("DIGITALREAD (INPUT BUFFER)");

    if (pin > 3) {
        DBG_PRINTLN("digitalRead: invalid pin index");
        return -1;
    }

    uint8_t buf[8];
    uint16_t len = sizeof(buf);

    // read GPIO buffer register instead of control
    if (!readGPIOBuffer(buf, len)) {
        DBG_PRINTLN("digitalRead: readGPIOBuffer failed");
        return -1;
    }

    // Response: buf[0]=Result, buf[1]=GPIO_BUF_HI, buf[2]=GPIO_BUF_LO
    uint8_t lo = buf[2];
    uint8_t mask = (1 << pin);

    int level = (lo & mask) ? HIGH : LOW;
    DBG_PRINT("Pin "); DBG_PRINT(pin); DBG_PRINT(" -> ");
    DBG_PRINTLN(level == HIGH ? "HIGH" : "LOW");
    return level;
}