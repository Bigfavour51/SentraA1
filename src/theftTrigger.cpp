#include "theftTrigger.h"

uint16_t lastTouched = 0;

static uint8_t relay_pin;
static uint32_t trigger_delay;

static uint32_t start_time = 0;

 bool waiting = false;
 bool triggered = false;

void TheftTrigger_Init(uint8_t relayPin, uint32_t delay_ms)
{
    relay_pin = relayPin;
    trigger_delay = delay_ms;

    pinMode(relay_pin, OUTPUT);
    digitalWrite(relay_pin, LOW);

    waiting = false;
    triggered = false;
}

void TheftTrigger_TouchDetected(void)
{
    if (!waiting && !triggered)
    {
        start_time = millis();
        waiting = true;
    }
}

void TheftTrigger_Update(void)
{
    if (waiting)
    {
        if (millis() - start_time >= trigger_delay)
        {
            digitalWrite(relay_pin, HIGH);
            triggered = true;
            waiting = false;
        }
    }
}

void TheftTrigger_Reset(void)
{
    digitalWrite(relay_pin, LOW);
    waiting = false;
    triggered = false;
}