#ifndef POWERCONTROLLER_H
#define POWERCONTROLLER_H

#include <Wire.h>
#include "Adafruit_HUSB238.h"

class PowerController {
public:
  // Costruttore: i pin SDA e SCL hanno valori di default (SDA=25, SCL=27)
  PowerController(uint8_t sdaPin = 25, uint8_t sclPin = 27);
  
  // Inizializza il chip HUSB238
  bool begin();
  
  // Seleziona la tensione richiesta e, contestualmente, la corrente.
  // Sono supportate esclusivamente 5V e 12V.
  // - Se si richiede 5V, il parametro current viene ignorato.
  // - Se si richiede 12V, current deve essere CURRENT_3_0_A oppure CURRENT_1_5_A.
  //   Se 12V non è disponibile o la corrente richiesta non corrisponde, viene fatto il fallback a 5V.
  // Se il puntatore fallbackOccurred non è nullptr, verrà impostato a true se il fallback è avvenuto.
  bool selectVoltage(HUSB238_PDSelection voltage, HUSB238_CurrentSetting current, bool* fallbackOccurred = nullptr);

private:
  Adafruit_HUSB238 husb238;  // Istanza del chip HUSB238
  uint8_t sdaPin;            // Pin SDA
  uint8_t sclPin;            // Pin SCL
};

#endif  // POWERCONTROLLER_H
