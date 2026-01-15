#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_MPR121.h>

/* ================== PINS ================== */
#define IR1_PIN        15
#define IR2_PIN        4
#define PI_INT_PIN     27
#define BUZZER_PIN     26

#define UART_TX_PIN    17   // ESP32 TX → Pi RX

/* ================== CONFIG ================== */
const float SENSOR_DISTANCE_M = 1.5;
const unsigned long TIMEOUT_MS = 3000;

/* ================== MPR121 ================== */
Adafruit_MPR121 cap;

/* ================== STATE ================== */
volatile unsigned long t_ir1 = 0;
volatile unsigned long t_ir2 = 0;
volatile bool ir1Triggered = false;
volatile bool ir2Triggered = false;

bool touchDetected = false;

/* ================== CRC16 ================== */
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

/* ================== ISR ================== */
void IRAM_ATTR ir1_isr() {
  if (!ir1Triggered) {
    t_ir1 = micros();
    ir1Triggered = true;
  }
}

void IRAM_ATTR ir2_isr() {
  if (ir1Triggered && !ir2Triggered) {
    t_ir2 = micros();
    ir2Triggered = true;
  }
}

/* ================== BUZZER ================== */
void beep(uint16_t ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

/* ================== PI INTERRUPT ================== */
void pulsePi() {
  digitalWrite(PI_INT_PIN, HIGH);
  delay(50);
  digitalWrite(PI_INT_PIN, LOW);
}

/* ================== UART PACKET ================== */
void send_uart_packet(float speed_kph, bool touch) {
  uint8_t packet[15];
  uint8_t idx = 0;

  packet[idx++] = 0xAA;       // SOF
  packet[idx++] = 0x01;       // Packet type

  memcpy(&packet[idx], &speed_kph, 4);
  idx += 4;

  packet[idx++] = touch ? 1 : 0;

  uint8_t eventFlags = 0x01;  // vehicle detected
  if (touch) eventFlags |= 0x04;
  packet[idx++] = eventFlags;

  uint32_t ts = millis();
  memcpy(&packet[idx], &ts, 4);
  idx += 4;

  uint16_t crc = crc16_ccitt(packet, idx);
  packet[idx++] = crc >> 8;
  packet[idx++] = crc & 0xFF;

  packet[idx++] = 0x55;       // EOF

  Serial1.write(packet, sizeof(packet));
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, -1, UART_TX_PIN);

  pinMode(IR1_PIN, INPUT_PULLUP);
  pinMode(IR2_PIN, INPUT_PULLUP);
  pinMode(PI_INT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(PI_INT_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(IR1_PIN), ir1_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR2_PIN), ir2_isr, FALLING);

  Wire.begin();
  cap.begin(0x5A);

  Serial.println("🚦 System ready");
}

/* ================== LOOP ================== */
void loop() {

  /* --- Touch detection --- */
  uint16_t touched = cap.touched();
  touchDetected = touched != 0;

  /* --- Vehicle detection --- */
  if (ir1Triggered) {

    if (ir2Triggered) {
      unsigned long delta_us = t_ir2 - t_ir1;
      float speed = (SENSOR_DISTANCE_M / (delta_us / 1e6)) * 3.6;

      Serial.print("Speed: ");
      Serial.print(speed);
      Serial.println(" km/h");

      send_uart_packet(speed, touchDetected);
      pulsePi();
      beep(100);

      ir1Triggered = false;
      ir2Triggered = false;
    }
    else if ((millis() - (t_ir1 / 1000)) > TIMEOUT_MS) {
      Serial.println("Timeout");
      beep(500);
      ir1Triggered = false;
      ir2Triggered = false;
    }
  }

  delay(5);
}
