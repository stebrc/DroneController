#include "LinMotor.h"

void setMotor(int extendPin, int retractPin, int extendState, int retractState) {
  digitalWrite(extendPin, extendState);
  digitalWrite(retractPin, retractState);
}

void controlMotors(float rollPID, float pitchPID) {
    // --- Gamba A ---
    if (!digitalRead(SWITCH_A)) {
        setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 1, 0); // Forza estensione
    } else {
        if (rollPID > 0)
            setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 0, 1); // Estensione (inverso)
        else if (rollPID < 0)
            setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 1, 0); // Ritrazione
        else
            setMotor(MOTOR_A_EXTEND, MOTOR_A_RETRACT, 0, 0); // Stop
    }

    // --- Gamba B ---
    if (!digitalRead(SWITCH_B)) {
        setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 1, 0);
    } else {
        if (pitchPID > 0)
            setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 1, 0);
        else if (pitchPID < 0)
            setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 0, 1);
        else
            setMotor(MOTOR_B_EXTEND, MOTOR_B_RETRACT, 0, 0);
    }

    // --- Gamba C ---
    if (!digitalRead(SWITCH_C)) {
        setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 1, 0);
    } else {
        if (rollPID > 0)
            setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 1, 0);
        else if (rollPID < 0)
            setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 0, 1);
        else
            setMotor(MOTOR_C_EXTEND, MOTOR_C_RETRACT, 0, 0);
    }

    // --- Gamba D ---
    if (!digitalRead(SWITCH_D)) {
        setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 1, 0);
    } else {
        if (pitchPID > 0)
            setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 0, 1); // Inverso
        else if (pitchPID < 0)
            setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 1, 0);
        else
            setMotor(MOTOR_D_EXTEND, MOTOR_D_RETRACT, 0, 0);
    }
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
