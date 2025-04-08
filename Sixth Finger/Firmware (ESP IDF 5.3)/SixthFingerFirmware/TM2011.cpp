#include "TM2011.h"
#include <Arduino.h>
#include "esp32-hal-gpio.h"

/**
 * @brief Costruttore della classe TM2011.
 *
 * Configura i pin relativi ai pulsanti interni ed esterni.
 */
TM2011::TM2011() {
  pinMode(cw_25deg_pin, INPUT);
  pinMode(push_pin, INPUT);
  pinMode(ccw_25deg_pin, INPUT);
  pinMode(ext_but_1_pin, INPUT_PULLUP);
  pinMode(ext_but_2_pin, INPUT_PULLUP);
}

/**
 * @brief Legge lo stato dei pulsanti esterni.
 *
 * Effettua la lettura digitale sui pin dei pulsanti esterni e restituisce lo stato
 * combinato dei pulsanti esterni.
 *
 * @return ButtonState Stato dei pulsanti esterni (EXT_BUT_1_AND_2, EXT_BUT_1, EXT_BUT_2 oppure UNKNOWN).
 */
ButtonState TM2011::readExternalButtonState() {
  int ext_but_1_state = digitalRead(ext_but_1_pin);
  int ext_but_2_state = digitalRead(ext_but_2_pin);
  if (!ext_but_1_state && !ext_but_2_state) {
    return EXT_BUT_1_AND_2;
  }
  if (!ext_but_1_state) {
    return EXT_BUT_1;
  }
  if (!ext_but_2_state) {
    return EXT_BUT_2;
  }
  return UNKNOWN;
}

/**
 * @brief Legge lo stato dei pulsanti interni.
 *
 * Effettua la lettura digitale sui pin dei pulsanti interni e restituisce lo stato corrispondente.
 *
 * @return ButtonState Stato dei pulsanti interni (CW_25DEG, PUSH, CCW_25DEG oppure UNKNOWN).
 */
ButtonState TM2011::readInternalButtonState() {
  int cw_25deg_state = digitalRead(cw_25deg_pin);
  int push_state = digitalRead(push_pin);
  int ccw_25deg_state = digitalRead(ccw_25deg_pin);

  if (cw_25deg_state == HIGH) {
    return CW_25DEG;
  } else if (push_state == HIGH) {
    return PUSH;
  } else if (ccw_25deg_state == HIGH) {
    return CCW_25DEG;
  } else {
    return UNKNOWN;
  }
}

/**
 * @brief Ottiene lo stato corrente dei pulsanti esterni.
 *
 * Legge lo stato dei pulsanti esterni e, se abilitato, stampa il risultato sul Serial Monitor per il debug.
 *
 * @param debug_print Se true, abilita la stampa del risultato.
 * @return ButtonState Stato corrente dei pulsanti esterni.
 */
ButtonState TM2011::getExternalButtonState(bool debug_print) {
  ButtonState currentState = readExternalButtonState();

  if (debug_print) {
    switch (currentState) {
      case EXT_BUT_1_AND_2:
        Serial.println("both buttons");
        break;
      case EXT_BUT_1:
        Serial.println("button1");
        break;
      case EXT_BUT_2:
        Serial.println("button2");
        break;
      case UNKNOWN:
        Serial.println("no button");
        break;
    }
  }

  return currentState;
}

/**
 * @brief Ottiene lo stato corrente dei pulsanti interni.
 *
 * Legge lo stato dei pulsanti interni e, se abilitato, stampa il risultato sul Serial Monitor per il debug.
 *
 * @param debug_print Se true, abilita la stampa del risultato.
 * @return ButtonState Stato corrente dei pulsanti interni.
 */
ButtonState TM2011::getInternalButtonState(bool debug_print) {
  ButtonState currentState = readInternalButtonState();

  if (debug_print) {
    switch (currentState) {
      case CW_25DEG:
        Serial.println("+25°");
        break;
      case PUSH:
        Serial.println("push!");
        break;
      case CCW_25DEG:
        Serial.println("-25°");
        break;
      case UNKNOWN:
        Serial.println("0°");
        break;
    }
  }

  return currentState;
}
