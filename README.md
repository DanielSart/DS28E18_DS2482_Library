# DS28E18 DS2482 Library

Arduino library for the **Maxim Integrated DS28E18** 1-Wire to I2C/SPI bridge, using the **DS2482-100/800** I2C to 1-Wire bridge as the controller.

This library provides a high-level API to interact with the DS28E18, including a sequencer builder to construct complex I2C/SPI transactions that the DS28E18 can execute autonomously.

## Features

- Full support for DS28E18 via DS2482 1-Wire master.
- **Sequencer Builder**: Easily construct I2C/SPI command sequences.
- **GPIO Control**: Configure and read/write the DS28E18 GPIO pins.
- **CRC16 Integrity**: Verification of 1-Wire communication.
- **ROM Commands**: Support for Search ROM, Match ROM, and Skip ROM.
- **PlatformIO Compatible**: Includes `library.json` for easy integration.

## Hardware Requirements

- **DS2482-100/800**: I2C to 1-Wire Bridge.
- **DS28E18**: 1-Wire to I2C/SPI Bridge.
- **Arduino Compatible Microcontroller**: (ESP32, RP2040, AVR, etc.)

## Installation

### PlatformIO

Add the following to your `platformio.ini`:

```ini
lib_deps =
    https://github.com/DanielSart/DS28E18_DS2482_Library.git
    adafruit/Adafruit DS2482
```

### Arduino IDE

1. Download this repository as a .zip file.
2. In the Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library...**.
3. Select the downloaded file.

## Quick Start

```cpp
#include <Wire.h>
#include <Adafruit_DS248x.h>
#include <DS28E18.h>

Adafruit_DS2482 ds2482;
DS28E18 bridge(ds2482);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ds2482.begin()) {
    Serial.println("DS2482 not found!");
    while (1);
  }

  if (!bridge.begin()) {
    Serial.println("DS28E18 not found!");
  }
}

void loop() {
  // Read GPIO status
  uint8_t hi, lo;
  if (bridge.getGPIOInputStates(hi, lo)) {
    Serial.print("GPIO High: "); Serial.println(hi, HEX);
    Serial.print("GPIO Low:  "); Serial.println(lo, HEX);
  }
  delay(1000);
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
