#include "Receiver.h"
#include <Arduino.h>

IBusBM ibus;

void initReceiver() {
  ibus.begin(Serial1);
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void readCommands(CommandInput &in) {
  in.roll     = readChannel(0, -50, 50, 0);
  in.pitch    = readChannel(1, -50, 50, 0) - 1;
  in.throttle = readChannel(2, 0, 100, 0);
  in.yaw      = readChannel(3, -50, 50, 0) + 1;
  in.Kroll    = readChannel(4, 0, 100, 0);
  in.Kpitch   = readChannel(5, 0, 100, 0);
  in.Kselect  = readChannel(6, 0, 2, 0);
  in.Kswitch  = readSwitch(7, 0);
  in.atterra  = readSwitch(8, 0);
}

void printReceiverInput(CommandInput &in) {
  Serial.print("inT: "); Serial.print(in.throttle);
  Serial.print(" | inP: "); Serial.print(in.pitch);
  Serial.print(" | inR: "); Serial.print(in.roll);
  Serial.print(" | inY: "); Serial.print(in.yaw);
  Serial.print(" | inKr: "); Serial.print(in.Kroll);
  Serial.print(" | inKp: "); Serial.print(in.Kpitch);
  Serial.print(" | inKs: "); Serial.print(in.Kselect);
  Serial.print(" | inKsw: "); Serial.println(in.Kswitch);
}