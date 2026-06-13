#include "speedDriver.h"

int sensorA {};
int sensorB {};

float distance {0.5f};      // 50cm bench test spacing
float speedValue {};
float overspeedTH {10.0f};

volatile unsigned long timeA {};
volatile unsigned long timeB {};

volatile unsigned long lastTriggerA {};
volatile unsigned long lastTriggerB {};

volatile bool triggeredA {};
volatile bool triggeredB {};

const uint32_t SENSOR_DEBOUNCE_US = 5000;      // 5ms
const uint32_t MAX_TRANSIT_US = 2000000;       // 2 sec timeout

void speedDriver_begin(int pinA, int pinB)
{
    sensorA = pinA;
    sensorB = pinB;

    pinMode(sensorA, INPUT_PULLUP);
    pinMode(sensorB, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(sensorA),
        speedDriver_sensorA_ISR,
        FALLING);

    attachInterrupt(
        digitalPinToInterrupt(sensorB),
        speedDriver_sensorB_ISR,
        FALLING);
}

void speedDriver_sensorA_ISR()
{
    unsigned long now = micros();

    if (now - lastTriggerA < SENSOR_DEBOUNCE_US)
        return;

    lastTriggerA = now;

    if (!triggeredA)
    {
        timeA = now;
        triggeredA = true;
    }
}

void speedDriver_sensorB_ISR()
{
    unsigned long now = micros();

    if (now - lastTriggerB < SENSOR_DEBOUNCE_US)
        return;

    lastTriggerB = now;

    if (triggeredA && !triggeredB)
    {
        timeB = now;
        triggeredB = true;
    }
}

bool speedDriver_speedAvailable()
{
    return (triggeredA && triggeredB);
}

float speedDriver_getSpeedKmh()
{
    if (timeB <= timeA)
        return 0.0f;

    float timeDiff = (timeB - timeA) / 1000000.0f;

    if (timeDiff <= 0.0f)
        return 0.0f;

    speedValue = (distance / timeDiff) * 3.6f;

    return speedValue;
}

void speedDriver_checkTimeout()
{
    if (triggeredA && !triggeredB)
    {
        if ((micros() - timeA) > MAX_TRANSIT_US)
        {
            speedDriver_reset();
        }
    }
}

void speedDriver_reset()
{
    triggeredA = false;
    triggeredB = false;

    timeA = 0;
    timeB = 0;
}