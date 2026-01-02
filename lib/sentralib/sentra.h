#ifndef __SENTRALIB_SENTRA_H__
#define __SENTRALIB_SENTRA_H__

#include <Arduino.h>
#include "uartMaster.h"

// GPIO definitions
#define buzzpin        12
#define relaypin       13
#define speedpin1      18
#define speedpin2      19
#define touchpin       4
#define interruptpin   23

// Config
extern const float SPEED_THRESHOLD; // km/h
extern const float SENSOR_DISTANCE;  // meters between IR sensors

// Shared state
extern volatile float vehicleSpeed;
extern volatile bool speedReady;

void sentra_setup();
void sentra_loop();

void sentra_set_relay(bool state);
bool sentra_get_touch();

#endif
