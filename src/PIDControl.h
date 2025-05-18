#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "Receiver.h"
#include "Utils.h"

#define MAX_PID_OUTPUT 400

extern const float Kp0, Ki0, Kd0;
extern const float Kp1, Ki1, Kd1;

struct PIDController {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevError;
  float threshold;
  float Ts;

  PIDController(float kp, float ki, float kd, float th, float ts)
    : Kp(kp), Ki(ki), Kd(kd), integral(0), prevError(0), threshold(th), Ts(ts) {}
};

void resetPID(PIDController &pid);
float computePID(PIDController &pid, float input, float setpoint);
void updatePID(CommandInput &in, PIDController &rollPID, PIDController &pitchPID, bool &lastSwitchState);
void printPIDParams(PIDController &pid);

#endif
