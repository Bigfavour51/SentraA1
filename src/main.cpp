#include <Arduino.h>
#include "speedDriver.h"
#include "uartDriver.h"

#define SENSOR_A PA0
#define SENSOR_B PA1
#define TOUCH_PIN PB6
#define RELAY_PIN PB0
#define ALARM_PIN PA6

void setup()
{
    uart_init(UART_BAUDRATE);

    speedDriver_begin( SENSOR_A,SENSOR_B);
    TheftTrigger_Init(TOUCH_PIN,RELAY_PIN,ALARM_PIN);

 

}

void loop()
{
    speedDriver_checkTimeout();

    if (speedDriver_speedAvailable())
    {
        float speed = speedDriver_getSpeedKmh();

        if (speed > overspeedTH)
        {
            uart_send_byte(speed, 0);
        }

        speedDriver_reset();
    }

    TheftTrigger_Update();
}
