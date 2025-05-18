#ifndef BRUSHLESS_H
#define BRUSHLESS_H

#include <Arduino.h>

// Pin motori
#define O1 2
#define O2 3
#define A1 5
#define A2 6

// Tick dei timer OCRx
#define ESC_STOP 1000 // Valore di "arming" (1ms, standard per ESC brushless)
#define ESC_MIN 1200
#define ESC_MAX 2000  // Valore di massima potenza (2ms)

// Calibra tutti i motori (ESC): da chiamare una volta nel setup
void calibrateESCs();

// Inizializza i timer per PWM a 50Hz
void initBrushlessPWM();

// Applica i valori PWM ai motori in base a PID e throttle
void updateMotors(int throttle, float rollPID, float pitchPID, float yawInput);

#endif
