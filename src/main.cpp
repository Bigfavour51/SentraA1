#include <Arduino.h>
#include <Wire.h>

// =====================================================
// ================== PIN DEFINITIONS ==================
// =====================================================
#define IR1_PIN PA4
#define IR2_PIN PA5

#define LED_PIN PC13           // Active LOW
#define PI_SERIAL Serial1      // PA9 TX, PA10 RX

// =====================================================
// ================== MPR121 CONFIG ====================
// =====================================================
#define MPR121_ADDR 0x5A
#define TOUCH_ELECTRODE 0

#define MPR121_TOUCH_STATUS_L 0x00
#define MPR121_ECR           0x5E

// =====================================================
// ================== SPEED CONFIG =====================
// =====================================================
#define IR_DISTANCE_METERS   0.15f
#define SPEED_LIMIT_KMH      30.0f

#define VEHICLE_TIMEOUT_US   1500000UL
#define LOCKOUT_US           3000000UL
#define IR_DEBOUNCE_US       200UL

#define IR_ACTIVE LOW

// =====================================================
// ================== STATE =============================
// =====================================================
enum TriggerState {
  IDLE,
  IR1_SEEN,
  IR2_SEEN
};

TriggerState state = IDLE;

unsigned long t1_us = 0;
unsigned long lastTriggerTime = 0;

bool lastTouchState = false;
bool currentTouch = false;

// =====================================================
// ================== I2C HELPERS ======================
// =====================================================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint16_t readTouchStatus() {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(MPR121_TOUCH_STATUS_L);
  Wire.endTransmission(false);
  Wire.requestFrom(MPR121_ADDR, (uint8_t)2);

  uint16_t status = Wire.read();
  status |= (Wire.read() << 8);
  return status;
}

// =====================================================
// ================== MPR121 INIT ======================
// =====================================================
void initMPR121() {
  writeRegister(MPR121_ECR, 0x00);

  writeRegister(0x41, 12); // Touch threshold
  writeRegister(0x42, 6);  // Release threshold

  writeRegister(0x2B, 0x01);
  writeRegister(0x2C, 0x01);
  writeRegister(0x2D, 0x00);
  writeRegister(0x2E, 0x00);

  writeRegister(0x2F, 0x01);
  writeRegister(0x30, 0x01);
  writeRegister(0x31, 0xFF);
  writeRegister(0x32, 0x02);

  writeRegister(MPR121_ECR, 0x01); // Enable electrode 0 only
}

// =====================================================
// ================== HELPERS ===========================
// =====================================================
inline bool ir1Active() {
  return digitalRead(IR1_PIN) == IR_ACTIVE;
}

inline bool ir2Active() {
  return digitalRead(IR2_PIN) == IR_ACTIVE;
}

void sendEvent(float speedKmh, bool touch) {
  PI_SERIAL.print("{\"event\":\"vehicle_detected\",\"speed_kmh\":");
  PI_SERIAL.print(speedKmh, 1);
  PI_SERIAL.print(",\"touch\":");
  PI_SERIAL.print(touch ? "true" : "false");
  PI_SERIAL.println(",\"source\":\"stm32-001\"}");
}

// =====================================================
// ================== SETUP =============================
// =====================================================
void setup() {
  pinMode(IR1_PIN, INPUT_PULLUP);
  pinMode(IR2_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED OFF

  PI_SERIAL.begin(115200);
  Serial.begin(115200);

  Wire.begin();        // PB6 / PB7
  Wire.setClock(100000);

  delay(100);
  initMPR121();

  Serial.println("STM32 Sentra Controller Ready");
}

// =====================================================
// ================== LOOP ==============================
// =====================================================
void loop() {
  unsigned long now_us = micros();

  // -------- Touch Handling --------
  uint16_t touchStatus = readTouchStatus();
  currentTouch = touchStatus & (1 << TOUCH_ELECTRODE);

  if (currentTouch && !lastTouchState) {
    digitalWrite(LED_PIN, LOW);   // LED ON
  } 
  else if (!currentTouch && lastTouchState) {
    digitalWrite(LED_PIN, HIGH);  // LED OFF
  }
  lastTouchState = currentTouch;

  // -------- Global lockout --------
  if (now_us - lastTriggerTime < LOCKOUT_US) return;

  // -------- Speed FSM --------
  switch (state) {

    case IDLE:
      if (ir1Active()) {
        delayMicroseconds(IR_DEBOUNCE_US);
        if (ir1Active()) {
          t1_us = micros();
          state = IR1_SEEN;
        }
      }
      else if (ir2Active()) {
        delayMicroseconds(IR_DEBOUNCE_US);
        if (ir2Active()) {
          t1_us = micros();
          state = IR2_SEEN;
        }
      }
      break;

    case IR1_SEEN:
      if (ir2Active()) {
        delayMicroseconds(IR_DEBOUNCE_US);
        if (!ir2Active()) break;

        float dt = (micros() - t1_us) / 1000000.0f;
        if (dt > 0.0001f) {
          float speed = (IR_DISTANCE_METERS / dt) * 3.6f;
          if (speed >= SPEED_LIMIT_KMH) {
            sendEvent(speed, currentTouch);
          }
        }

        lastTriggerTime = micros();
        state = IDLE;
      }
      else if (now_us - t1_us > VEHICLE_TIMEOUT_US) {
        state = IDLE;
      }
      break;

    case IR2_SEEN:
      if (ir1Active()) {
        delayMicroseconds(IR_DEBOUNCE_US);
        if (!ir1Active()) break;

        float dt = (micros() - t1_us) / 1000000.0f;
        if (dt > 0.0001f) {
          float speed = (IR_DISTANCE_METERS / dt) * 3.6f;
          if (speed >= SPEED_LIMIT_KMH) {
            sendEvent(speed, currentTouch);
          }
        }

        lastTriggerTime = micros();
        state = IDLE;
      }
      else if (now_us - t1_us > VEHICLE_TIMEOUT_US) {
        state = IDLE;
      }
      break;
  }
}
