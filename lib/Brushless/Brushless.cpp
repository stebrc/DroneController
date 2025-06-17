#include "Brushless.h"

void initBrushlessPWM() {
    pinMode(O1, OUTPUT);
    pinMode(O2, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);

    // === Timer3 (pin 2, 3, 5) ===
    TCCR3A = 0; TCCR3B = 0;
    TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);
    TCCR3B |= (1 << WGM33) | (1 << CS31); // Prescaler 8, Phase-Correct PWM
    ICR3 = 20000; // 100Hz: 16MHz / (2 * 8 * 20000) ≈ 50Hz (Phase-Correct)

    OCR3B = ESC_STOP;
    OCR3C = ESC_STOP;
    OCR3A = ESC_STOP;

    // === Timer4 (pin 6) ===
    TCCR4A = 0; TCCR4B = 0;
    TCCR4A |= (1 << COM4A1) | (1 << WGM41);
    TCCR4B |= (1 << WGM43) | (1 << CS41); // Prescaler 8, Phase-Correct PWM
    ICR4 = 20000; // Stessa frequenza

    OCR4A = ESC_STOP;
}

void updateMotors(int throttle, float rollPID, float pitchPID, float yawPID) {
  if (throttle > 0){
    // Combinazione dei PID + input yaw
    int throttleO1 = throttle - rollPID - pitchPID - yawPID; // O
    int throttleO2 = throttle + rollPID + pitchPID - yawPID; // O
    int throttleA1 = throttle - rollPID + pitchPID + yawPID; // A
    int throttleA2 = throttle + rollPID - pitchPID + yawPID; // A

    // Limita tra 0 e 100
    throttleO1 = constrain(throttleO1, 0, 100);
    throttleO2 = constrain(throttleO2, 0, 100);
    throttleA1 = constrain(throttleA1, 0, 100);
    throttleA2 = constrain(throttleA2, 0, 100);

    // Applica i PWM
    OCR3B = map(throttleO1, 0, 100, ESC_MIN, ESC_MAX); // pin 2 (O1)
    OCR3C = map(throttleO2, 0, 100, ESC_MIN, ESC_MAX); // pin 3 (O2)
    OCR3A = map(throttleA1, 0, 100, ESC_MIN, ESC_MAX); // pin 5 (A1)
    OCR4A = map(throttleA2, 0, 100, ESC_MIN, ESC_MAX); // pin 6 (A2)
  }
  else{
    OCR3B = ESC_STOP;
    OCR3C = ESC_STOP;
    OCR3A = ESC_STOP;
    OCR4A = ESC_STOP;
  }
}

void calibrateESCs() {
  // Massimo segnale
  OCR3B = ESC_MAX;
  OCR3C = ESC_MAX;
  OCR3A = ESC_MAX;
  OCR4A = ESC_MAX;
  delay(3000);

  // Minimo segnale
  OCR3B = ESC_STOP;
  OCR3C = ESC_STOP;
  OCR3A = ESC_STOP;
  OCR4A = ESC_STOP;
  delay(3000);
}
