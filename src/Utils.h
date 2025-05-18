#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>  // Include Arduino core se necessario

// Funzioni per il debugging

void printAttitudeInfo(float roll, float pitch, float rollSetpoint, float pitchSetpoint, 
                       float rollAction, float pitchAction, float yawAction, float distance);

// Funzioni matematiche/utilità
float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

#endif // UTILS_H