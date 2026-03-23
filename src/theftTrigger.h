#ifndef THEFT_TRIGGER_H
#define THEFT_TRIGGER_H

#include <Arduino.h>
#include <Adafruit_MPR121.h>
#include <Wire.h>

extern uint16_t lastTouched;

// Pins & config
static uint8_t relay_pin;
static uint8_t alarm_pin;
static uint32_t trigger_delay;

// Timing
extern  uint32_t start_time;
extern  uint32_t reset_time;

// States
extern  bool waiting;
extern  bool triggered;
extern  bool isTouched;

// Debounce
extern  uint32_t last_touch_time;

#define TOUCH_DEBOUNCE_MS 100
#define ACTIVE_TIME_MS 3000   // replaces delay(3000)


void TheftTrigger_Init(uint8_t relayPin, uint8_t alarmpin, uint32_t delay_ms);

void TheftTrigger_TouchDetected(void);

void TheftTrigger_Update(void);

void TheftTrigger_Reset(void);

void detectTouch(void);

#endif