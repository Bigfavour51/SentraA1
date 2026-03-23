#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <Arduino.h>
#include "speedDriver.h"
#include "theftTrigger.h"

#define UART_BAUDRATE 115200
#define PIserial Serial1

void uart_init(unsigned long baudrate);
void uart_send_byte(float speed_kmh, uint8_t theft_trigger_state);


#endif // UART_DRIVER_H