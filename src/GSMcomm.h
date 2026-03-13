#ifndef GSMCOMM_H
#define GSMCOMM_H

#include <Arduino.h>
#include <SoftwareSerial.h>


    void GSMComm(int rxPin, int txPin);
    void begin(long baud);
    bool sendSMS(const char *number, const char *message);
    String readSMS();
    void deleteSMS();



#endif