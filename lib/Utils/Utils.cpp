#include "Utils.h"

void printAttitudeInfo(float roll, float pitch, float rollSetpoint, float pitchSetpoint, 
                       float rollAction, float pitchAction, float yawAction, float distance) {
  Serial.print("R: "); Serial.print(roll);
  Serial.print(" | P: "); Serial.print(pitch);
  Serial.print(" | RSP: "); Serial.print(rollSetpoint);
  Serial.print(" | PSP: "); Serial.print(pitchSetpoint);
  Serial.print(" | RPID: "); Serial.print(rollAction);
  Serial.print(" | PPID: "); Serial.print(pitchAction);
  Serial.print(" | YPID: "); Serial.print(yawAction);
  Serial.print(" | DFG: "); Serial.println(distance);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}