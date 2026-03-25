#include <Arduino.h>
#include <Adafruit_MPR121.h>
// #include <Wire.h>

// #define MPR121_ADDR 0x5A

// uint16_t lastState = 0;

// Adafruit_MPR121 sensor = Adafruit_MPR121();

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();

//   if (!sensor.begin(MPR121_ADDR)) {
//     Serial.println("MPR121 not found");
//     while (1);
//   }

//   Serial.println("MPR121 Test Starting...");
// }

// void loop() {
//   uint16_t touchState = sensor.touched();

//   // Check electrode 0 (bit 0)
//   if (touchState & 0x0001) {
//     Serial.println("TOUCHED");
//   } else {
//     Serial.println("RELEASED");
//   }

//   delay(100);
// }

// #include <Wire.h>

// // Minimal stub for readFilteredData to avoid undefined identifier.
// // Replace with actual MPR121 reading code when integrating the sensor.
// uint16_t readFilteredData(uint8_t electrode) {
//   (void)electrode; // suppress unused parameter warning
//   return 0;
// }

// void setup() {
//   Wire.begin();
//   Serial.begin(115200);
//   Serial.println("Scanning...");

//   for (byte addr = 1; addr < 127; addr++) {
//     Wire.beginTransmission(addr);
//     if (Wire.endTransmission() == 0) {
//       Serial.print("Found: 0x");
//       Serial.println(addr, HEX);
//     }
//   }
// }

// void loop() {
//   uint16_t raw = readFilteredData(1);

//   Serial.print("RAW: ");
//   Serial.println(raw);

//   delay(100);
// }

#include <Wire.h>
#include <Adafruit_MPR121.h>

Adafruit_MPR121 cap = Adafruit_MPR121();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("MPR121 RAW + TOUCH TEST");

  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found!");
    while (1);
  }

  Serial.println("MPR121 initialized");

  // VERY LOW thresholds (to force detection)
  cap.setThresholds(2, 1);
}

void loop() {
uint16_t raw = cap.filteredData(0);
uint16_t baseline = cap.baselineData(0);

Serial.print("RAW: ");
Serial.print(raw);

Serial.print(" | BASE: ");
Serial.print(baseline);

Serial.print(" | DIFF: ");
Serial.print(baseline - raw);

Serial.print(" | TOUCH: ");
if (cap.touched() & (1 << 0)) {
  Serial.println("YES");
} else {
  Serial.println("NO");
}
delay(700);
}