#ifndef POWERCONTROLLER_H
#define POWERCONTROLLER_H

#include <Wire.h>
#include "Adafruit_HUSB238.h"

class PowerController {
public:
  // Costruttore: i pin SDA e SCL hanno valori di default (SDA=25, SCL=27)
  PowerController(uint8_t sdaPin = 25, uint8_t sclPin = 27);
  
  // Inizializza il chip HUSB238 e il pin TPS2121
  bool begin();
  
  // Seleziona la tensione richiesta e, contestualmente, la corrente.
  bool selectVoltage(HUSB238_PDSelection voltage, HUSB238_CurrentSetting current, bool* fallbackOccurred = nullptr);
  
  // Avvia un task FreeRTOS che legge ogni 5 secondi il pin TPS2121 e stampa il valore
  void startTPS2121Task();

  // Pin di stato del TPS2121
  uint8_t status_TPS2121_Pin = 26;
  Adafruit_HUSB238 husb238;  // Istanza del chip HUSB238

private:
  uint8_t sdaPin;            // Pin SDA
  uint8_t sclPin;            // Pin SCL
  
};

#endif  // POWERCONTROLLER_H
