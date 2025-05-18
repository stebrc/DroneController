#include "Infrared.h"

// Costruttore
Infrared::Infrared(uint16_t interval_ms) {
    interval = interval_ms;
    lastReadTime = 0;
}

// Inizializzazione del sensore
void Infrared::begin() {
    Wire.begin();
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Errore: VL53L0X non trovato!");
        while (1);
    }

    // Impostazioni opzionali per lungo raggio
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

    sensor.startContinuous();  // Modalità continua
}

// Verifica se è passato il tempo per leggere
bool Infrared::isTimeToRead() {
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= interval) {
        lastReadTime = currentTime;
        return true;
    }
    return false;
}

// Lettura della distanza
uint16_t Infrared::readDistance() {
    return sensor.readRangeContinuousMillimeters();
}
