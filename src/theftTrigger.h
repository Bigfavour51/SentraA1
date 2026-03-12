#ifndef THEFT_TRIGGER_H
#define THEFT_TRIGGER_H

#include <Arduino.h>
#include <Adafruit_MPR121.h>
#include <Wire.h>

extern uint16_t lastTouched;
extern bool waiting;
extern bool triggered;

void TheftTrigger_Init(uint8_t relayPin, uint32_t delay_ms);

void TheftTrigger_TouchDetected(void);

void TheftTrigger_Update(void);

void TheftTrigger_Reset(void);

#endif