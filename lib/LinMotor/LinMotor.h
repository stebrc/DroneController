#ifndef LINMOTOR_H
#define LINMOTOR_H

#include <Arduino.h>

#define MOTOR_A_EXTEND 40  
#define MOTOR_A_RETRACT 38 
#define MOTOR_B_EXTEND 52  
#define MOTOR_B_RETRACT 50 
#define MOTOR_C_EXTEND 48  
#define MOTOR_C_RETRACT 46  
#define MOTOR_D_EXTEND 44  
#define MOTOR_D_RETRACT 42
#define SWITCH_A 65
#define SWITCH_B 66
#define SWITCH_C 64
#define SWITCH_D 67

void initLinMotor();

void setMotor(char motor, int command);

void controlMotors(float rollPID, float pitchPID);

void extendUntilContact(bool &vola, bool &atterra);

void retractAllMotors();

#endif // LINMOTOR_H
