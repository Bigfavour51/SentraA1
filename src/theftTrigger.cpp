#include "theftTrigger.h"

Adafruit_MPR121 touch;

uint16_t lastTouched = 0;

// Timing
uint32_t start_time {};
uint32_t reset_time {};

// States
 bool waiting = false;
 bool triggered = false;
 bool isTouched = false;

// Debounce
uint32_t last_touch_time = 0;
#define TOUCH_DEBOUNCE_MS 100
#define ACTIVE_TIME_MS 3000   // replaces delay(3000)



void TheftTrigger_Init(uint8_t relayPin, uint8_t alarmpin, uint32_t delay_ms)
{
    relay_pin = relayPin;
    alarm_pin = alarmpin;
    trigger_delay = delay_ms;

    pinMode(relay_pin, OUTPUT);
    pinMode(alarm_pin, OUTPUT);

    digitalWrite(relay_pin, LOW);
    digitalWrite(alarm_pin, LOW);

    if (!touch.begin(0x5A))
    {
        // Serial.println("Touch sensor not initialized");
        while(1);
    }
   
    waiting = false;
    triggered = false;
}

void detectTouch(void)
{
   
    uint16_t touched = touch.touched();
    uint32_t now = millis();

    for (uint8_t i = 0; i < 12; i++)
    {
        bool nowTouched = touched & (1 << i);
        bool wasTouched = lastTouched & (1 << i);

        // New touch with debounce
        if (nowTouched && !wasTouched)
        {
            if (now - last_touch_time > TOUCH_DEBOUNCE_MS)
            {
                last_touch_time = now;

                
                    isTouched = true;
                
                if (!waiting && !triggered)
                {
                    start_time = now;
                    waiting = true;
                }
            }
        }
    }

    lastTouched = touched;

    TheftTrigger_Update();
}

void TheftTrigger_Update(void)
{
    uint32_t now = millis();

    // Waiting before trigger
    if (waiting && (now - start_time >= trigger_delay))
    {
        digitalWrite(relay_pin, HIGH);
        digitalWrite(alarm_pin, HIGH);

        triggered = true;
        waiting = false;

        reset_time = now; // start active timer
    }

    // Auto reset after active period
    if (triggered && (now - reset_time >= ACTIVE_TIME_MS))
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