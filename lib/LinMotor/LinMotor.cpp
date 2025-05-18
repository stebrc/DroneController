#include "LinMotor.h"

void setMotor(int extendPin, int retractPin, int extendState, int retractState) {
  digitalWrite(extendPin, extendState);
  digitalWrite(retractPin, retractState);
}

void controlMotors(float rollPID, float pitchPID) {

    int switchA = digitalRead(SWITCH_A);
    int switchB = digitalRead(SWITCH_B);
    int switchC = digitalRead(SWITCH_C);
    int switchD = digitalRead(SWITCH_D);

    if (pitchPID > 0) {
        setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 1, 0);
        setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 0, 1);
    } else if (pitchPID < 0) {
        setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 0, 1);
        setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 1, 0);
    } else {
        setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 0, 0);
        setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 0, 0);
    }

    if (rollPID > 0) {
        setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 0, 1);
        setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 1, 0);
    } else if (rollPID < 0) {
        setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 1, 0);
        setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 0, 1);
    } else {
        setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 0, 0);
        setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 0, 0);
    }

    if (!switchA) setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 1, 0);
    if (!switchB) setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 1, 0);
    if (!switchC) setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 1, 0);
    if (!switchD) setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 1, 0);
}

void extendUntilContact(bool &vola, bool &atterra){
    setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 1, 0);
    setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 1, 0);
    setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 1, 0);
    setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 1, 0);

    // Lettura dei microswitch
    int switchA = digitalRead(SWITCH_A);
    int switchB = digitalRead(SWITCH_B);
    int switchC = digitalRead(SWITCH_C);
    int switchD = digitalRead(SWITCH_D);

    if (switchA && switchB && switchC && switchD){
    vola = false;
    atterra = false;
    }
}