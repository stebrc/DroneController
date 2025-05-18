#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Wire.h>
#include <math.h>

struct mpuData {
    float accX, accY, accZ;
    float rateRoll, ratePitch, rateYaw;
    float biasRoll = 0, biasPitch = 0, biasYaw = 0;
    float biasAccX = 0, biasAccY = 0, biasAccZ = 0;
};

class Orientation {
public:
    Orientation(unsigned long dt_ms);
    void begin();
    void calibrate();
    bool isDataReady();
    void update(float& pitch, float& roll);  // <--- nuova funzione

    mpuData data;
    
private:    
    unsigned long dt_us;
    unsigned long previousMicros;
    float AngleRoll = 0.0, AnglePitch = 0.0;

    float KalmanStateRoll = 0.0, KalmanUncertaintyRoll = 2.0;
    float KalmanStatePitch = 0.0, KalmanUncertaintyPitch = 2.0;

    const float w = 4;  // Rumore di processo
    const float v = 3;  // Rumore di misura

    void readSensor();
    void kalmanFilter(float& state, float& uncertainty, float input, float measurement);
};

#endif
