#ifndef ONEWIRE_BUS_H
#define ONEWIRE_BUS_H

#include <Arduino.h>
#include <Adafruit_DS248x.h>
#include "DS28E18.h"

#define ONEWIREBUS_MAX_DEVICES 20

class OneWireBus {
public:
    explicit OneWireBus(Adafruit_DS248x& ds);

    // Initialise DS2482 and shared DS28E18 interface
    bool begin(bool enCRC = true);

    // ----- Device registration -----
    bool addDevice(const uint8_t rom[8]);
    uint8_t numDevices() const { return deviceCount; }

    // Retrieve ROM of a device
    bool getDeviceROM(uint8_t index, uint8_t outRom[8]) const;

    // ----- Global operations -----
    bool skipROM();                             // Enter global Skip‑ROM mode
    bool matchROM();                            // Select one device by ROM
    bool initializeROMs();                      // Run ROM init via Skip‑ROM
    bool resetAllDeviceStatus();                // Global reset device status

    // ----- Access a specific device -----
    DS28E18& device(uint8_t index);              // Automatically Match‑ROM

    // ----- Debug Control -----
    void enableDebug() {DS28E18_Debug = true;};
    void disableDebug() {DS28E18_Debug = false;};

    // ----- Overdrive control -----
    void enableOverdrive() {
        ds28e18.enableOverdrive();
    }
    void disableOverdrive() {
        ds28e18.disableOverdrive();
    }
    bool overdriveEnabled() const { return ds28e18.overdriveEnabled(); }

private:
    Adafruit_DS248x& ds;
    DS28E18 ds28e18;

    uint8_t deviceROMs[ONEWIREBUS_MAX_DEVICES][8];
    uint8_t deviceCount;

    // Internal helpers
    void setActiveROM(const uint8_t rom[8]);
};

#endif