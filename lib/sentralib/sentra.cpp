#include <Arduino.h>
#include "sentra.h"
#include "uartMaster.h"

/* ================= CONFIGURATION ================= */

// GPIO definitions (ESP32-safe)
#define buzzpin        12
#define relaypin       13
#define speedpin1      18    // IR Sensor A
#define speedpin2      19    // IR Sensor B
#define touchpin       4
#define interruptpin   23    // Output to Raspberry Pi

// Speed config
const float SPEED_THRESHOLD = 60.0;   // km/h
const float SENSOR_DISTANCE = 5.0;    // meters between IR sensors

/* ================= INTERNAL STATE ================= */

volatile unsigned long tStart = 0;
volatile unsigned long tEnd   = 0;
volatile bool waitingForSecondSensor = false;

volatile float vehicleSpeed = 0.0;
volatile bool speedReady = false;

/* ================= INTERRUPT ROUTINES ================= */

// First IR sensor (vehicle enters)
void IRAM_ATTR sensorA_ISR()
{
    if (!waitingForSecondSensor)
    {
        tStart = micros();
        waitingForSecondSensor = true;
    }
}

// Second IR sensor (vehicle exits)
void IRAM_ATTR sensorB_ISR()
{
    if (waitingForSecondSensor)
    {
        tEnd = micros();
        unsigned long deltaTime = tEnd - tStart;

        if (deltaTime > 0)
        {
            float time_sec = deltaTime / 1e6;
            vehicleSpeed = (SENSOR_DISTANCE / time_sec) * 3.6; // km/h
            speedReady = true;
        }

        waitingForSecondSensor = false;
    }
}

/* ================= SETUP ================= */

void sentra_setup()
{
    Serial.begin(115200);
    uart_master_setup();

    pinMode(relaypin, OUTPUT);
    pinMode(touchpin, INPUT);
    pinMode(speedpin1, INPUT_PULLUP);
    pinMode(speedpin2, INPUT_PULLUP);
    pinMode(buzzpin, OUTPUT);
    pinMode(interruptpin, OUTPUT);

    digitalWrite(relaypin, HIGH);       // relay OFF (active low)
    digitalWrite(buzzpin, LOW);         // buzzer OFF
    digitalWrite(interruptpin, LOW);    // Pi interrupt LOW

    attachInterrupt(digitalPinToInterrupt(speedpin1), sensorA_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(speedpin2), sensorB_ISR, RISING);

    // Power-on beep
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(buzzpin, HIGH);
        delay(300);
        digitalWrite(buzzpin, LOW);
        delay(300);
    }
}

/* ================= MAIN LOOP ================= */

void sentra_loop()
{
    if (speedReady)
    {
        Serial.print("Vehicle speed: ");
        Serial.print(vehicleSpeed);
        Serial.println(" km/h");

        bool overspeed = (vehicleSpeed > SPEED_THRESHOLD);

        if (overspeed)
        {
            // Send event to Raspberry Pi
            uart_send_event(vehicleSpeed, true);

            // Pulse interrupt pin (IMPORTANT)
            digitalWrite(interruptpin, HIGH);
            delay(10);                  // 10 ms pulse
            digitalWrite(interruptpin, LOW);
        }

        // Reset for next vehicle
        vehicleSpeed = 0.0;
        speedReady = false;
    }
}

/* ================= UTILITIES ================= */

bool sentra_get_touch()
{
    return digitalRead(touchpin) == HIGH;
}

void sentra_set_relay(bool state)
{
    if (state)
    {
        digitalWrite(relaypin, LOW);   // relay ON
        delay(5000);
        digitalWrite(relaypin, HIGH);  // relay OFF
    }
    else
    {
        digitalWrite(relaypin, HIGH);  // relay OFF
    }
}
