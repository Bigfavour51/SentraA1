#include "uartMaster.h"

void uart_master_setup()
{
    Serial2.begin(BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);
    delay(200);
    Serial.println("UART READY (Sentra)");
}

void uart_send_event(float speed, bool overspeed)
{
    // Format: #EVT,<flag>,<speed>\n
    Serial2.print("#EVT,");
    Serial2.print(overspeed ? 1 : 0);
    Serial2.print(",");
    Serial2.print(speed, 2);
    Serial2.print("\n");

    // Optional debug
    Serial.print("UART TX: ");
    Serial.print("#EVT,");
    Serial.print(overspeed ? 1 : 0);
    Serial.print(",");
    Serial.println(speed, 2);
}
