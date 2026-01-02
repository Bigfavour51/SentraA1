#ifndef __UARTMASTER_H__
#define __UARTMASTER_H__

#include <Arduino.h>

// UART configuration
#define TXPIN     17
#define RXPIN     16
#define BAUDRATE  9600

void uart_master_setup();
void uart_send_event(float speed, bool overspeed);

#endif
