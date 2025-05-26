#include "Orientation.h"
#include <Arduino.h>

Orientation::Orientation(unsigned long dt_ms) {
    dt_us = dt_ms * 1000;
    previousMicros = micros();
}

void Orientation::begin() {
    Wire.begin();
    Wire.setClock(400000);
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);      // Power management register
    Wire.write(0x00);      // Wake up MPU6050
    Wire.endTransmission();
}

void Orientation::calibrate() {
    const int samples = 2000;

    // accelerometro
    data.biasAccX = 0;
    data.biasAccY = 0;
    data.biasAccZ = 0;

    // giroscopio
    data.biasRoll = 0;
    data.biasPitch = 0;
    data.biasYaw = 0;

    for (int i = 0; i < samples; i++) {
        readSensor();
        data.biasAccX += data.accX;
        data.biasAccY += data.accY;
        data.biasAccZ += data.accZ - 1;
        data.biasRoll += data.rateRoll;
        data.biasPitch += data.ratePitch;
        data.biasYaw += data.rateYaw;
        delay(1);
    }

    data.biasAccX /= samples;
    data.biasAccY /= samples;
    data.biasAccZ /= samples;

    data.biasRoll /= samples;
    data.biasPitch /= samples;
    data.biasYaw /= samples;

    // Serial.print(data.biasAccX);
    // Serial.print(",");
    // Serial.print(data.biasAccY);
    // Serial.print(",");
    // Serial.println(data.biasAccZ); 
}

bool Orientation::isDataReady() {
    unsigned long now = micros();
    if (now - previousMicros >= dt_us) {
        previousMicros = now;
        return true;
    }
    return false;
}

void Orientation::readSensor() {
    // Set low-pass filter to 5Hz
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // Set accelerometer range to ±8g
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Read accelerometer data
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    int16_t accXRaw = Wire.read() << 8 | Wire.read();
    int16_t accYRaw = Wire.read() << 8 | Wire.read();
    int16_t accZRaw = Wire.read() << 8 | Wire.read();

    data.accX = (float)accXRaw / 4096.0f;
    data.accY = (float)accYRaw / 4096.0f;
    data.accZ = (float)accZRaw / 4096.0f;

    // Set gyro range to ±500 °/s
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // Read gyro data
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    int16_t gyroXRaw = Wire.read() << 8 | Wire.read();
    int16_t gyroYRaw = Wire.read() << 8 | Wire.read();
    int16_t gyroZRaw = Wire.read() << 8 | Wire.read();

    data.rateRoll = ((float)gyroXRaw / 65.5f);
    data.ratePitch = ((float)gyroYRaw / 65.5f);
    data.rateYaw = ((float)gyroZRaw / 65.5f);
}

void Orientation::kalmanFilter(float& state, float& uncertainty, float input, float measurement) {
    float dt = dt_us / 1e6;

    state += dt * input;
    uncertainty += dt * dt * w * w;

    float kalmanGain = uncertainty / (uncertainty + v * v);

    state = state + kalmanGain * (measurement - state);
    uncertainty = (1 - kalmanGain) * uncertainty;
}

void Orientation::update(float& pitch, float& roll) {
    readSensor();

    // Serial.print(data.accX);
    // Serial.print(",");
    // Serial.print(data.accY);
    // Serial.print(",");
    // Serial.println(data.accZ); 

    data.accX -= data.biasAccX;
    data.accY -= data.biasAccY;
    data.accZ -= data.biasAccZ;    

    data.rateRoll -= data.biasRoll;
    data.ratePitch -= data.biasPitch;
    data.rateYaw -= data.biasYaw; 

    AngleRoll = atan2(data.accY, sqrt(data.accX * data.accX + data.accZ * data.accZ)) * 180.0 / PI;
    AnglePitch = -atan2(data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) * 180.0 / PI;

    kalmanFilter(KalmanStateRoll, KalmanUncertaintyRoll, data.rateRoll, AngleRoll);
    kalmanFilter(KalmanStatePitch, KalmanUncertaintyPitch, data.ratePitch, AnglePitch);
    roll = KalmanStateRoll;
    pitch = KalmanStatePitch;
}
