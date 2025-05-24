#ifndef INFRARED_H
#define INFRARED_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class Infrared {
public:
    Infrared(uint16_t interval_ms = 100);  // Costruttore con intervallo di lettura
    void begin();                          // Inizializza il sensore
    bool isTimeToRead();                   // Ritorna true se sono passati 100 ms
    uint16_t readDistance();               // Ritorna la distanza in mm

private:
    VL53L0X sensor;
    uint16_t interval;
    unsigned long lastReadTime;
};

#endif
