#include <Arduino.h>

#include <Orientation.h>
#include <PIDControl.h>
#include <Receiver.h>
#include <Infrared.h>
#include <Brushless.h>
#include <LinMotor.h>
#include <Utils.h>

/* === Timing === */
const float dt = 0.008;

/* === Orientamento === */
Orientation imu(1000*dt);  // Lettura rapida dei dati IMU
float pitch, roll;

/* === PID === */
const float rollThreshold = 0;
const float pitchThreshold = 0;

const float Kp0 = 0.6, Ki0 = 0.35, Kd0 = 0.03;
const float Kp1 = 6, Ki1 = 3.5, Kd1 = 0.3;

PIDController flightRollPID(2, 0, 0, rollThreshold, dt);   // Kp, Ki, Kd, soglia, Ts
PIDController flightPitchPID(2, 0, 0, pitchThreshold, dt);

PIDController flightRateRollPID(Kp0, Ki0, Kd0, rollThreshold, dt);   // Kp, Ki, Kd, soglia, Ts
PIDController flightRatePitchPID(Kp0, Ki0, Kd0, pitchThreshold, dt);
PIDController flightRateYawPID(2, 12, 0, rollThreshold, dt);   // Kp, Ki, Kd, soglia, Ts

PIDController groundRollPID(1, 0.1, 1, rollThreshold, dt);
PIDController groundPitchPID(1, 0.1, 1, rollThreshold, dt);

bool lastSwitchState = false; // Per modificare i guadagni online

float rollSetpoint = 0.0;
float pitchSetpoint = 0.0;
float yawRateSetpoint = 0.0;

float rollPID, pitchPID, yawPID;

/* === Ricevitore === */
CommandInput in;

/* === Infrarossi === */
Infrared ir(100);  // Lettura lenta dei dati IR

/* === Stato === */
bool vola = true, lastAtterra, atterra;
uint16_t distance = 65535;

/* === Prototipi loop principale === */
void flightControlLoop();
void groundControlLoop();

void setup() {
  Serial.begin(115200);

  initReceiver();         // inizializza ricevitore iBus
  initBrushlessPWM();     // inizializza PWM per ESC

  ir.begin();             // inizializza Infrarossi
  imu.begin();

  imu.calibrate();
  // calibra gli ESC
}

void loop() {
  if (ir.isTimeToRead()) {
    distance = ir.readDistance();
  }
  if (imu.isDataReady()) {
    // Aggiorna l'orientamento stimato
    imu.update(pitch, roll);

    // Leggi input dal radiocomando
    readCommands(in);

    // Prepara l'atterraggio solo se è abbastanza vicino al suolo
    atterra = in.atterra && !lastAtterra;

    if (atterra && distance < 200) {  // Estende le gambe fino al contatto con lo switch
      extendUntilContact(vola, atterra);
    } 
    if(vola || atterra){  // Se sta volando o atterrando
      flightControlLoop();
    }
    else {  // Se è già atterrato
      groundControlLoop();
    }

    // Debug (seriale)
    printReceiverInput(in);
    printAttitudeInfo(roll, pitch, rollSetpoint, pitchSetpoint, rollPID, pitchPID, yawPID, distance / 10.0);

    lastAtterra = in.atterra;
  }
}

void flightControlLoop() {
  // Aggiorna i guadagni PID
  updatePID(in, flightRateRollPID, flightRatePitchPID, lastSwitchState);

  // Aggiorna i setpoint
  rollSetpoint = in.roll;
  pitchSetpoint = in.pitch;
  yawRateSetpoint = in.yaw;

  // Calcola le velocità angolari desiderate
  float desiredRollRate = computePID(flightRollPID, roll, rollSetpoint);
  float desiredPitchRate = computePID(flightPitchPID, pitch, pitchSetpoint);

  rollPID = computePID(flightRateRollPID, imu.data.rateRoll, desiredRollRate);
  pitchPID = computePID(flightRatePitchPID, imu.data.ratePitch, desiredPitchRate);
  yawPID = computePID(flightRateYawPID, imu.data.rateYaw, yawRateSetpoint);

  // Applica il PWM ai motori
  updateMotors(in.throttle, rollPID, pitchPID, yawPID);
}

void groundControlLoop() {
  // Calcola le azioni PID
  rollPID = computePID(groundRollPID, roll, 0);
  pitchPID = computePID(groundPitchPID, pitch, 0);

  // Controlla i motori
  controlMotors(rollPID, pitchPID); 

  if(groundRollPID.prevError == 0 && groundPitchPID.prevError == 0){
    vola = true;
  }    
}