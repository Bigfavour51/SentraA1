#ifndef SPEEDTRAP_H
#define SPEEDTRAP_H

#include <Arduino.h>

extern int sensorA;
extern int sensorB;

extern float distance; // meters between sensors
extern float speedValue;
extern float overspeedTH;

extern volatile unsigned long timeA;
extern volatile unsigned long timeB;

extern volatile bool triggeredA;
extern volatile bool triggeredB;



void speedDriver_begin(int pinA, int pinB);
void speedDriver_sensorA_ISR();
void speedDriver_sensorB_ISR();
bool speedDriver_speedAvailable();
float speedDriver_getSpeedKmh();
void speedDriver_reset();

#endif