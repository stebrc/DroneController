#include <Arduino.h>
#include <EEPROM.h>    // Includi la libreria EEPROM
#include <avr/wdt.h>   // Includi la libreria per il Watchdog Timer

#include <Orientation.h>
#include <PIDControl.h>
#include <Receiver.h>
#include <Infrared.h>
#include <Brushless.h>
#include <LinMotor.h>
#include <Utils.h>

/* === Timing === */
const float dt = 0.02;

/* === Orientamento === */
Orientation imu(1000*dt);  // Lettura rapida dei dati IMU
float pitch, roll;

/* === PID === */
// Soglie per il controllo in volo
const float rollThreshold = 0;
const float pitchThreshold = 0;
const float rollRateThreshold = 0.5;
const float pitchRateThreshold = 0.5;
const float yawRateThreshold = 0.5;

// Soglie per il controllo a terra
const float gRollThreshold = 1.5;
const float gPitchThreshold = 1.5;

const float Kp0 = 0.6, Ki0 = 0.035, Kd0 = 0.03;
const float Kp1 = 6, Ki1 = 0.35, Kd1 = 0.3;

PIDController flightRollPID(2, 0, 0, rollThreshold, dt);   // Kp, Ki, Kd, soglia, Ts
PIDController flightPitchPID(2, 0, 0, pitchThreshold, dt);

PIDController flightRateRollPID(Kp0, Ki0, Kd0, rollRateThreshold, dt);
PIDController flightRatePitchPID(Kp0, Ki0, Kd0, pitchRateThreshold, dt);
PIDController flightRateYawPID(2, 12, 0, yawRateThreshold, dt);

PIDController groundRollPID(0.1, 0.0, 0, gRollThreshold, dt);
PIDController groundPitchPID(0.1, 0.0, 0, gPitchThreshold, dt);

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
bool vola = false, lastInAtterra = false, atterra;
uint16_t distance = 65535;

/* === Prototipi loop principale === */
void flightControlLoop();
void groundControlLoop();

// --- Struttura per salvare i bias IMU in EEPROM ---
struct IMUBiasData {
    float biasRoll;
    float biasPitch;
    float biasYaw;
    float biasAccX;
    float biasAccY;
    float biasAccZ;
    byte calibrationFlag; // Flag per indicare se la calibrazione è stata fatta
};

const int EEPROM_ADDRESS = 0; // Indirizzo di partenza per salvare i dati IMU nella EEPROM
const byte CALIBRATION_DONE_FLAG = 0xA5; // Valore "magico" per il flag

void setup() {
  Serial.begin(115200);
  // Disabilita il watchdog timer temporaneamente per evitare reset durante il setup
  wdt_disable(); 

  Serial.println("Avvio DroneController...");

  // iBus
  initReceiver();

  // IR
  ir.begin();        
  
  // LA (Linear Actuators)
  initLinMotor();

  // IMU
  imu.begin();

  // --- Calibrazione IMU con EEPROM ---
  IMUBiasData storedBiasData;
  EEPROM.get(EEPROM_ADDRESS, storedBiasData); // Leggi i dati dalla EEPROM

  if (storedBiasData.calibrationFlag == CALIBRATION_DONE_FLAG) {
    // Se il flag è presente, carica i bias dalla EEPROM
    imu.data.biasRoll = storedBiasData.biasRoll;
    imu.data.biasPitch = storedBiasData.biasPitch;
    imu.data.biasYaw = storedBiasData.biasYaw;
    imu.data.biasAccX = storedBiasData.biasAccX;
    imu.data.biasAccY = storedBiasData.biasAccY;
    imu.data.biasAccZ = storedBiasData.biasAccZ;
    Serial.println("IMU bias caricati da EEPROM.");
  } else {
    // Se il flag non è presente, esegui la calibrazione e salvala
    Serial.println("Calibrando IMU (prima volta o dopo reset 'pulito')..."); 
    retractAllMotors();
    delay(3000);
    imu.calibrate();
    
    // Salva i bias calibrati nella struttura
    storedBiasData.biasRoll = imu.data.biasRoll;
    storedBiasData.biasPitch = imu.data.biasPitch;
    storedBiasData.biasYaw = imu.data.biasYaw;
    storedBiasData.biasAccX = imu.data.biasAccX;
    storedBiasData.biasAccY = imu.data.biasAccY;
    storedBiasData.biasAccZ = imu.data.biasAccZ;
    storedBiasData.calibrationFlag = CALIBRATION_DONE_FLAG; // Imposta il flag di calibrazione
    
    // Scrivi la struttura completa nella EEPROM
    EEPROM.put(EEPROM_ADDRESS, storedBiasData);
    Serial.println("IMU calibrata e bias salvati in EEPROM.");
  }

  // ESC (Brushless Motors)
  initBrushlessPWM();
  // calibrateESCs(); 

  // Abilita il watchdog timer
  wdt_enable(WDTO_30MS); 
  Serial.println("Watchdog Timer abilitato.");
  Serial.println("Setup completato. Avvio loop principale.");
}

void loop() {
  // Reset del watchdog timer in ogni iterazione del loop
  wdt_reset(); 

  if (ir.isTimeToRead()) {
    distance = ir.readDistance();
  }
  if (imu.isDataReady()) {
    // Aggiorna l'orientamento stimato
    imu.update(pitch, roll);

    // Leggi input dal radiocomando
    readCommands(in); 

    // Aggiorna i setpoint
    rollSetpoint = in.roll;
    pitchSetpoint = in.pitch;
    yawRateSetpoint = in.yaw;

    // Prepara l'atterraggio quando viene commutato lo switch
    // atterra = in.atterra && !lastInAtterra;
    // lastInAtterra = in.atterra;

    if (distance < 160) {  // Estende le gambe fino al contatto con lo switch
      extendUntilContact(vola, atterra);
    } 
    else {  // Ritira le gambe quando non c'è contatto
      retractAllMotors();
    }

    if(vola){  // Se sta volando o atterrando
      flightControlLoop();
    }
    else {  // Se è a terra
      groundControlLoop();
    }

    // Debug (seriale)
    printReceiverInput(in);
    printAttitudeInfo(roll, pitch, rollSetpoint, pitchSetpoint, rollPID, pitchPID, yawPID, distance / 10.0);
  }
}

void flightControlLoop() {
  // Aggiorna i guadagni PID
  updatePID(in, flightRateRollPID, flightRatePitchPID, lastSwitchState);

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
  rollPID = computePID(groundRollPID, roll, rollSetpoint);
  pitchPID = computePID(groundPitchPID, pitch, pitchSetpoint);

  // Controlla i motori
  controlMotors(rollPID, pitchPID); 

  if(abs(groundRollPID.prevError) < 2 && abs(groundPitchPID.prevError) < 2) {
    vola = true;
  }    
}