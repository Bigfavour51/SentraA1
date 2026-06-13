#include "theftTrigger.h"

// Pins
static uint8_t touch_pin;
static uint8_t relay_pin;
static uint8_t alarm_pin;

// Configuration
uint32_t trigger_delay = 2000;

// Timing
static uint32_t start_time = 0;
static uint32_t reset_time = 0;
static uint32_t last_touch_time = 0;

// States
bool waiting = false;
bool triggered = false;
bool isTouched = false;

// Constants
#define TOUCH_DEBOUNCE_MS 100
#define ACTIVE_TIME_MS    5000

void TheftTrigger_Init(uint8_t touchPin, uint8_t relayPin,uint8_t alarmPin)
{
    touch_pin = touchPin;
    relay_pin = relayPin;
    alarm_pin = alarmPin;

    pinMode(touch_pin, INPUT);

    pinMode(relay_pin, OUTPUT);
    pinMode(alarm_pin, OUTPUT);

    digitalWrite(relay_pin, LOW);
    digitalWrite(alarm_pin, LOW);

       for(size_t i = 0; i < 5; i++)
    {
        digitalWrite(relay_pin, HIGH);
        delay(200);
        digitalWrite(relay_pin, LOW);
        delay(200);
    }

    waiting = false;
    triggered = false;
    isTouched = false;
}

void TheftTrigger_Update(void)
{
    static bool lastTouchState = false;

    uint32_t now = millis();

    //--------------------------------------------------
    // Touch Detection
    //--------------------------------------------------

    bool touchState = digitalRead(touch_pin);

    if (touchState && !lastTouchState)
    {
        if ((now - last_touch_time) > TOUCH_DEBOUNCE_MS)
        {
            last_touch_time = now;

            isTouched = true;

            if (!waiting && !triggered)
            {
                waiting = true;
                start_time = now;
            }
        }
    }

    lastTouchState = touchState;

    //--------------------------------------------------
    // Trigger Delay
    //--------------------------------------------------

    if (waiting && ((now - start_time) >= trigger_delay))
    {
        
        digitalWrite(relay_pin, HIGH);
        digitalWrite(alarm_pin, HIGH);

        waiting = false;
        triggered = true;

        reset_time = now;
    }

    //--------------------------------------------------
    // Active Period
    //--------------------------------------------------

    if (triggered && ((now - reset_time) >= ACTIVE_TIME_MS))
    {
        TheftTrigger_Reset();
    }
}

void TheftTrigger_Reset(void)
{
    digitalWrite(relay_pin, LOW);
    digitalWrite(alarm_pin, LOW);

    waiting = false;
    triggered = false;
    isTouched = false;
}