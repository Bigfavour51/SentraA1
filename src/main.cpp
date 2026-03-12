#include <Arduino.h>
#include "speedDriver.h"
#include "theftTrigger.h"

#define IR_A PA0
#define IR_B PA1
#define RELAY_PIN PB0

Adafruit_MPR121 touch;

void setup()
{
    Serial.begin(9600);
    speedDriver_begin(IR_A, IR_B);
    Wire.begin();
    
    if (!touch.begin(0x5A))
    {
        Serial.println("Touch sensor not initialized");
        while(1);
    }
    delay(1000);
    touch.setThresholds(12, 6);
    TheftTrigger_Init(RELAY_PIN, 1000);
}


void loop()
{
    if (speedDriver_speedAvailable())
    {
        float speed = speedDriver_getSpeedKmh();

        Serial.print("Speed: ");
        Serial.print(speed);
        Serial.println(" km/h");

        if (speed > overspeedTH)
        {
            Serial.println("OVER SPEED!");
        }
        speedDriver_reset();


        uint16_t touched = touch.touched();

        for (uint8_t i = 0; i < 12; i++)
        {
            bool nowTouched = touched & (1 << i);
            bool wasTouched = lastTouched & (1 << i);

            // new touch
            if (nowTouched && !wasTouched)
            {
                Serial.println("Touch detected");
                TheftTrigger_TouchDetected();
            }

            // release event
            if (!nowTouched && wasTouched)
            {
                Serial.println("Touch released");
            }
        }

        lastTouched = touched;

        TheftTrigger_Update();
        if (triggered)
        {
            delay(3000);
             TheftTrigger_Reset();
        }

    }
}



