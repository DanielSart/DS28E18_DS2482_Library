#include "OneWireBus.h"

OneWireBus::OneWireBus(Adafruit_DS248x& ds248x)
    : ds(ds248x), ds28e18(ds248x), deviceCount(0), lastActiveDevice(-1) {}

bool OneWireBus::begin(bool enCRC) {
    return ds28e18.begin(enCRC);
}

bool OneWireBus::addDevice(const uint8_t rom[8]) {
    if (deviceCount >= ONEWIREBUS_MAX_DEVICES) return false;
    memcpy(deviceROMs[deviceCount++], rom, 8);
    ds28e18.setROM(rom);
    return true;
}

bool OneWireBus::getDeviceROM(uint8_t index, uint8_t outRom[8]) const {
    if (index >= deviceCount) return false;
    memcpy(outRom, deviceROMs[index], 8);
    return true;
}

bool OneWireBus::skipROM() {
    lastActiveDevice = -1;  // Invalidate: skip mode, no specific device
    return ds28e18.skipROM();
}

bool OneWireBus::matchROM() {
    return ds28e18.matchROM();
}

void OneWireBus::setActiveROM(const uint8_t rom[8]) {
    ds28e18.setROM(rom);
    ds28e18.matchROM();
}

DS28E18& OneWireBus::device(uint8_t index) {
    if (index >= deviceCount) {
        // fallback: default skip mode in case of invalid index
        ds28e18.skipROM();
        lastActiveDevice = -1;  // Invalidate: skip mode, no specific device
    } else if (lastActiveDevice != (int8_t)index) {
        // Only re-select if switching to a different device
        setActiveROM(deviceROMs[index]);
        lastActiveDevice = (int8_t)index;
    }
    // If same device is already active, skip redundant ROM selection
    return ds28e18;
}

void OneWireBus::invalidateActiveDevice() {
    lastActiveDevice = -1;
}

bool OneWireBus::initializeROMs() {
    // Perform the GPIO initialisation sequence on all chips
    if (!skipROM()) return false;
    return ds28e18.initializeGPIO();
}

bool OneWireBus::resetAllDeviceStatus() {
    if (!skipROM()) return false;
    return ds28e18.resetDeviceStatus();
}