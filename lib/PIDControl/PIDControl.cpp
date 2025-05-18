#include "PIDControl.h"
#include <Arduino.h>

void resetPID(PIDController &pid) {
  pid.integral = 0;
  pid.prevError = 0;
}

float computePID(PIDController &pid, float input, float setpoint) {
  float error = setpoint - input;

  if (abs(error) < pid.threshold) error = 0;

  pid.integral += (error + pid.prevError) / 2 * pid.Ts;
  float derivative = (error - pid.prevError) / pid.Ts;
  pid.prevError = error;

  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;

  if (output < -MAX_PID_OUTPUT || output > MAX_PID_OUTPUT) {
    pid.integral -= error * pid.Ts; // Prevent integral windup
  }

  return constrain(output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

void updatePID(CommandInput &in, PIDController &rollPID, PIDController &pitchPID, bool &lastSwitchState) {
  if (in.Kswitch && !lastSwitchState){
    switch(in.Kselect){
      case 0:     
        rollPID.Kp = mapFloat(in.Kroll, 0, 100, Kp0, Kp1);
        pitchPID.Kp = mapFloat(in.Kpitch, 0, 100, Kp0, Kp1);
        break;
      case 1:
        rollPID.Ki = mapFloat(in.Kroll, 0, 100, Ki0, Ki1);
        pitchPID.Ki = mapFloat(in.Kpitch, 0, 100, Ki0, Ki1);
        break;
      case 2:
        rollPID.Kd = mapFloat(in.Kroll, 0, 100, Kd0, Kd1);
        pitchPID.Kd = mapFloat(in.Kpitch, 0, 100, Kd0, Kd1);
        break;
    }
  }
  lastSwitchState = in.Kswitch;
}

void printPIDParams(PIDController &pid){
  Serial.print("KPr: "); Serial.print(pid.Kp);
  Serial.print(" | KIr: "); Serial.print(pid.Ki);
  Serial.print(" | KDr: "); Serial.print(pid.Kd);
  Serial.print(" | KPp: "); Serial.print(pid.Kp);
  Serial.print(" | KIp: "); Serial.print(pid.Ki);
  Serial.print(" | KDp: "); Serial.println(pid.Kd);
}