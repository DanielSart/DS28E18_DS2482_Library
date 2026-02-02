#include <Wire.h>
#include <Adafruit_DS248x.h>
#include <DS28E18.h>

Adafruit_DS248x ds2482;
DS28E18 ds28e18(ds2482);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  DS28E18_Debug = false;

  Serial.println("Starting DS28E18 ROM Readout...");

  if (!ds2482.begin(&Wire, 0x18)) {
    Serial.println("DS2482 not found!");
    while (1);
  }

  if (!ds28e18.begin(true)) {
    Serial.println("DS28E18 init failed!");
    while (1);
  }

  ds28e18.skipROM();

  // Configure GPIOs (I2C Pullups on DS28E18 side)
  if (!ds28e18.initializeGPIO()) {
    Serial.println("DS28E18 GPIO Init Failed");
    while(1);
  }

  uint8_t rom[8];

  // single call replaces the old manual Table 68 sequence
  if (!ds28e18.readROM(rom)) {
    Serial.println("Read ROM failed");
  }

  Serial.print("ROM: ");
    for (int i = 0; i < 8; i++) {
      if (rom[i] < 16) {
        Serial.print("0");
      }
      Serial.print(rom[i], HEX);
      Serial.print(" ");
    }
  Serial.println();
}

void loop() {

}