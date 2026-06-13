#ifndef THEFT_TRIGGER_H
#define THEFT_TRIGGER_H

#include <Arduino.h>

// Configuration
extern uint32_t trigger_delay;

// States
extern bool waiting;
extern bool triggered;
extern bool isTouched;

// API
void TheftTrigger_Init(uint8_t touchPin,
                       uint8_t relayPin,
                       uint8_t alarmPin);

void TheftTrigger_Update(void);

void TheftTrigger_Reset(void);

#endif