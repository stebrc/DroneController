#ifndef RECEIVER_H
#define RECEIVER_H

#include <IBusBM.h>
#include <Arduino.h>

// Struct che raccoglie tutti gli input da iBus
struct CommandInput {
  int8_t roll;
  int8_t pitch;
  uint8_t throttle;
  int8_t yaw;
  uint8_t Kroll;
  uint8_t Kpitch;
  uint8_t Kselect;
  uint8_t Kswitch;
  uint8_t atterra;
};

// Inizializza il ricevitore iBus (es: Serial1)
void initReceiver();

// Legge un canale mappandolo in un intervallo definito
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);

// Legge un interruttore on/off da un canale iBus
bool readSwitch(byte channelInput, bool defaultValue = false);

// Popola la struct con tutti i comandi dal radiocomando
void readCommands(CommandInput &in);

// Stampa i valori letti dal ricevitore
void printReceiverInput(CommandInput &in);

#endif
