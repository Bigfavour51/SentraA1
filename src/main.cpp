#include <Arduino.h>
#include "sentra.h"
#include "uartMaster.h"



void setup() {
  // put your setup code here, to run once:

  sentra_setup();
 
}

void loop() {
  // put your main code here, to run repeatedly:
  sentra_loop();
}

