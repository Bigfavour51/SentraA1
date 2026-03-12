#include "speedDriver.h"

int sensorA {};
int sensorB {};
float distance {0.5}; // meters between sensors
float speedValue {};
float overspeedTH {10.0};

volatile unsigned long timeA {};
volatile unsigned long timeB {};

volatile bool triggeredA {};
volatile bool triggeredB {};



void speedDriver_begin(int pinA, int pinB)
{
    sensorA = pinA;
    sensorB = pinB;

    pinMode(sensorA, INPUT);
    pinMode(sensorB, INPUT);

    attachInterrupt(digitalPinToInterrupt(sensorA), speedDriver_sensorA_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(sensorB), speedDriver_sensorB_ISR, FALLING);
}

void speedDriver_sensorA_ISR()
{
    if (!triggeredA)
    {
        timeA = micros();
        triggeredA = true;
    }
}

void speedDriver_sensorB_ISR()
{
    if (triggeredA && !triggeredB)
    {
        timeB = micros();
        triggeredB = true;
    }
}

bool speedDriver_speedAvailable()
{
    return (triggeredA && triggeredB);
}

float speedDriver_getSpeedKmh()
{
    float timeDiff = (timeB - timeA) / 1000000.0;
    float speed = distance / timeDiff;
    speedValue = speed * 3.6;
    return speedValue;
}

void speedDriver_reset()
{
    triggeredA = false;
    triggeredB = false;
}