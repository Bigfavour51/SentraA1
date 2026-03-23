#include "uartDriver.h"




void uart_init(unsigned long baudrate)
{
    PIserial.begin(baudrate);
}

void uart_send_byte(float speed_kmh, uint8_t theft_trigger_state)
{
    PIserial.print("{\"event\":\"vehicle_detected\",");
    
    PIserial.print("\"speed_kmh\":");
    PIserial.print(speed_kmh, 1);
    PIserial.print(",");

    PIserial.print("\"touch\":");
    PIserial.print(theft_trigger_state ? "true" : "false");
    PIserial.print(",");

    PIserial.print("\"source\":\"stm32-001\"");

    PIserial.println("}");
}

