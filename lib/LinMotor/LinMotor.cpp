#include "LinMotor.h"

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
    bool doneA = false, doneB = false, doneC = false, doneD = false;

    while (!(doneA && doneB && doneC && doneD)) {
        if (!doneA) {
            if (digitalRead(SWITCH_A)) {
                setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 0, 0);
                doneA = true;
            } else {
                setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 1, 0);
            }
        }

        if (!doneB) {
            if (digitalRead(SWITCH_B)) {
                setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 0, 0);
                doneB = true;
            } else {
                setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 1, 0);
            }
        }

        if (!doneC) {
            if (digitalRead(SWITCH_C)) {
                setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 0, 0);
                doneC = true;
            } else {
                setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 1, 0);
            }
        }

        if (!doneD) {
            if (digitalRead(SWITCH_D)) {
                setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 0, 0);
                doneD = true;
            } else {
                setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 1, 0);
            }
        }
    }

    vola = false;
    atterra = false;
}
