#include "LinMotor.h"

void initLinMotor(){
    pinMode(MOTOR_A_ENABLE, OUTPUT);
    pinMode(MOTOR_B_ENABLE, OUTPUT);
    pinMode(MOTOR_C_ENABLE, OUTPUT);
    pinMode(MOTOR_D_ENABLE, OUTPUT);

    pinMode(MOTOR_A_EXTEND, OUTPUT);
    pinMode(MOTOR_A_RETRACT, OUTPUT);
    pinMode(MOTOR_B_EXTEND, OUTPUT);
    pinMode(MOTOR_B_RETRACT, OUTPUT);
    pinMode(MOTOR_C_EXTEND, OUTPUT);
    pinMode(MOTOR_C_RETRACT, OUTPUT);
    pinMode(MOTOR_D_EXTEND, OUTPUT);
    pinMode(MOTOR_D_RETRACT, OUTPUT);

    pinMode(SWITCH_A, INPUT_PULLUP);
    pinMode(SWITCH_B, INPUT_PULLUP);
    pinMode(SWITCH_C, INPUT_PULLUP);
    pinMode(SWITCH_D, INPUT_PULLUP);

    enableAllMotors();
    retractAllMotors();
    delay(3000);
}

void enableAllMotors() {
    digitalWrite(MOTOR_A_ENABLE, HIGH);
    digitalWrite(MOTOR_B_ENABLE, HIGH);
    digitalWrite(MOTOR_C_ENABLE, HIGH);
    digitalWrite(MOTOR_D_ENABLE, HIGH);
}

void disableAllMotors() {
    digitalWrite(MOTOR_A_ENABLE, LOW);
    digitalWrite(MOTOR_B_ENABLE, LOW);
    digitalWrite(MOTOR_C_ENABLE, LOW);
    digitalWrite(MOTOR_D_ENABLE, LOW);

    setMotor('A', 0);
    setMotor('B', 0);
    setMotor('C', 0);
    setMotor('D', 0);
}

void retractAllMotors() {

    setMotor('A', -1);
    setMotor('B', -1);
    setMotor('C', -1);
    setMotor('D', -1);
}

void setMotor(char motor, int command) {
    int extendPin = -1, retractPin = -1;
    switch (motor) {
        case 'A':
            extendPin = MOTOR_A_EXTEND;
            retractPin = MOTOR_A_RETRACT;
            break;
        case 'B':
            extendPin = MOTOR_B_EXTEND;
            retractPin = MOTOR_B_RETRACT;
            break;
        case 'C':
            extendPin = MOTOR_C_EXTEND;
            retractPin = MOTOR_C_RETRACT;
            break;
        case 'D':
            extendPin = MOTOR_D_EXTEND;
            retractPin = MOTOR_D_RETRACT;
            break;
        default:
            return; // Motore non valido
    }

    switch (command) {
        case -1: // Ritrazione
            digitalWrite(extendPin, LOW);
            digitalWrite(retractPin, HIGH);
            break;
        case 0: // Stallo
            digitalWrite(extendPin, LOW);
            digitalWrite(retractPin, LOW);
            break;
        case 1: // Estensione
            digitalWrite(extendPin, HIGH);
            digitalWrite(retractPin, LOW);
            break;
        default:
            digitalWrite(extendPin, LOW);
            digitalWrite(retractPin, LOW);
            break;
    }
}

void controlMotors(float rollPID, float pitchPID) {
    auto applyControl = [](char motor, int sw, float pid, bool invert = false) {
        int command = 0;
        if (!digitalRead(sw)) {
            command = 1; // Estendi se non premuto
        } else {
            if (pid > 0) command = invert ? -1 : 1;
            else if (pid < 0) command = invert ? 1 : -1;
            else command = 0;
        }
        setMotor(motor, command);
    };

    applyControl('A', SWITCH_A, rollPID, true);
    applyControl('B', SWITCH_B, pitchPID);
    applyControl('C', SWITCH_C, rollPID);
    applyControl('D', SWITCH_D, pitchPID, true);
}

void extendUntilContact(bool &vola, bool &atterra) {
    bool swA = digitalRead(SWITCH_A);
    bool swB = digitalRead(SWITCH_B);
    bool swC = digitalRead(SWITCH_C);
    bool swD = digitalRead(SWITCH_D);

    // Serial.print("A:"); Serial.print(swA);
    // Serial.print(" B:"); Serial.print(swB);
    // Serial.print(" C:"); Serial.print(swC);
    // Serial.print(" D:"); Serial.println(swD);

    setMotor('A', swA ? 0 : 1);
    setMotor('B', swB ? 0 : 1);
    setMotor('C', swC ? 0 : 1);
    setMotor('D', swD ? 0 : 1);

    if (swA && swB && swC && swD) {
        vola = false;
        atterra = false;
    }
}