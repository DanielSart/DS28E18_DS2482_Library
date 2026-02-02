#include <Wire.h>
#include <Adafruit_DS248x.h>
#include <DS28E18.h>

Adafruit_DS248x ds2482;
DS28E18 ds28e18(ds2482);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  DS28E18_Debug = false;

  Serial.println("Starting DS28E18 GPIO test...");

  if (!ds2482.begin(&Wire, 0x18)) {
    Serial.println("DS2482 not found!");
    while (1);
  }

  if (!ds28e18.begin(true)) {
    Serial.println("DS28E18 init failed!");
    while (1);
  }

  ds28e18.skipROM();

  // single call replaces the old manual Table 68 sequence
  if (!ds28e18.initializeGPIO()) {
    Serial.println("GPIO init failed");
  }

  ds28e18.pinMode(GPIOA, WEAK_PULLUP);
  ds28e18.pinMode(GPIOB, WEAK_PULLUP);
}

void loop() {
  ds28e18.digitalWrite(GPIOA, HIGH);
  Serial.println("GPIOA set HIGH");
  delay(500);
  ds28e18.digitalWrite(GPIOA, LOW);
  Serial.println("GPIOA set LOW");
  delay(500);

  int v = ds28e18.digitalRead(GPIOB);
  Serial.print("GPIOB = ");
  Serial.println(v);
  delay(500);
}