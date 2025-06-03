#include "LinMotor.h"

void initLinMotor(){
    pinMode(MOTOR_A_EXTEND, OUTPUT);
    pinMode(MOTOR_A_RETRACT, OUTPUT);
    pinMode(MOTOR_B_EXTEND, OUTPUT);
    pinMode(MOTOR_B_RETRACT, OUTPUT);
    pinMode(MOTOR_C_EXTEND, OUTPUT);
    pinMode(MOTOR_C_RETRACT, OUTPUT);
    pinMode(MOTOR_D_EXTEND, OUTPUT);
    pinMode(MOTOR_D_RETRACT, OUTPUT);

    pinMode(SWITCH_A, INPUT);
    pinMode(SWITCH_B, INPUT);
    pinMode(SWITCH_C, INPUT);
    pinMode(SWITCH_D, INPUT);

    retractAllMotors();
    delay(2000);
}

void setMotor(int extendPin, int retractPin, int extendState, int retractState) {
  digitalWrite(extendPin, extendState);
  digitalWrite(retractPin, retractState);
}

void controlMotors(float rollPID, float pitchPID) {
    auto applyControl = [](int sw, float pid, int ext, int ret, bool invert = false) {
        int extend = 0, retract = 0;
        if (!digitalRead(sw)) {
            extend = 1;
            retract = 0;
        } else {
            if (pid > 0) {
                extend = invert ? 0 : 1;
                retract = invert ? 1 : 0;
            } else if (pid < 0) {
                extend = invert ? 1 : 0;
                retract = invert ? 0 : 1;
            }
        }
        setMotor(ext, ret, extend, retract);
    };

    applyControl(SWITCH_A, rollPID, MOTOR_A_EXTEND, MOTOR_A_RETRACT, true);
    applyControl(SWITCH_B, pitchPID, MOTOR_B_EXTEND, MOTOR_B_RETRACT);
    applyControl(SWITCH_C, rollPID, MOTOR_C_EXTEND, MOTOR_C_RETRACT);
    applyControl(SWITCH_D, pitchPID, MOTOR_D_EXTEND, MOTOR_D_RETRACT, true);
}

void extendUntilContact(bool &vola, bool &atterra) {
    bool swA = digitalRead(SWITCH_A);
    bool swB = digitalRead(SWITCH_B);
    bool swC = digitalRead(SWITCH_C);
    bool swD = digitalRead(SWITCH_D);

    setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, !swA, 0);
    setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, !swB, 0);
    setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, !swC, 0);
    setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, !swD, 0);

    if (swA && swB && swC && swD) {
        vola = false;
        atterra = false;
    }
}

void retractAllMotors() {
    setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 0, 1);
    setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 0, 1);
    setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 0, 1);
    setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 0, 1);
}