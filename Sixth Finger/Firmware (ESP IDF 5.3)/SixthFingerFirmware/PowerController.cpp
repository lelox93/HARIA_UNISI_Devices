#include "PowerController.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

PowerController::PowerController(uint8_t sdaPin, uint8_t sclPin)
  : sdaPin(sdaPin), sclPin(sclPin) {}

bool PowerController::begin() {
  Wire.begin(sdaPin, sclPin);

  // Imposta il pin TPS2121 come input
  pinMode(status_TPS2121_Pin, INPUT);

  // Inizializza il chip HUSB238
  if (husb238.begin(HUSB238_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("HUSB238 initialized successfully.");
    return true;
  }
  Serial.println("Couldn't find HUSB238, check your wiring?");
  return false;
}

bool PowerController::selectVoltage(HUSB238_PDSelection voltage, HUSB238_CurrentSetting current, bool* fallbackOccurred) {
  if (!husb238.isAttached()) {
    Serial.println("No PD device attached.");
    return false;
  }

  if (husb238.getPDResponse() != SUCCESS) {
    Serial.println("Failed to get PD response.");
    return false;
  }

  // Gestione per 5V: il parametro current viene ignorato
  if (voltage == PD_SRC_5V) {
    if (!husb238.isVoltageDetected(PD_SRC_5V)) {
      Serial.println("5V not available.");
      return false;
    }
    if (fallbackOccurred) *fallbackOccurred = false;
    Serial.println("Setting PD voltage: 5V");
    husb238.selectPD(PD_SRC_5V);
    husb238.requestPD();
    return true;
  }
  // Gestione per 12V
  else if (voltage == PD_SRC_12V) {
    if (husb238.isVoltageDetected(PD_SRC_12V)) {
      HUSB238_CurrentSetting availableCurrent = husb238.currentDetected(PD_SRC_12V);
      if (availableCurrent == current) {
        if (fallbackOccurred) *fallbackOccurred = false;
        Serial.print("Setting PD voltage: 12V with ");
        if (current == CURRENT_3_0_A) {
          Serial.println("3A");
        } else if (current == CURRENT_1_5_A) {
          Serial.println("1.5A");
        }
        husb238.selectPD(PD_SRC_12V);
        husb238.requestPD();
        return true;
      } else {
        Serial.println("Requested 12V current not available.");
      }
    } else {
      Serial.println("12V not available.");
    }

    // Fallback a 5V
    if (husb238.isVoltageDetected(PD_SRC_5V)) {
      if (fallbackOccurred) *fallbackOccurred = true;
      Serial.println("Fallback: setting PD voltage to 5V.");
      husb238.selectPD(PD_SRC_5V);
      husb238.requestPD();
      return true;
    } else {
      Serial.println("Fallback not available: 5V not available.");
      return false;
    }
  } else {
    Serial.println("Unsupported voltage. Only 5V and 12V are supported.");
    return false;
  }
}

// Funzione task per la lettura periodica del pin TPS2121
void TPS2121Task(void* param) {
  // Recupera l'istanza di PowerController passata come parametro
  PowerController* self = static_cast<PowerController*>(param);

  for (;;) {
    int pinValue = digitalRead(self->status_TPS2121_Pin);
    Serial.print("TPS2121 Status Pin value: ");
    Serial.println(pinValue);
    self->husb238.selectPD(PD_SRC_12V);
    self->husb238.requestPD();
    // Attendi 5000 ms
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// Avvia il task FreeRTOS per la lettura del pin TPS2121
void PowerController::startTPS2121Task() {
  Serial.println("Creating TPS2121 Task!");
  
  xTaskCreate(
    TPS2121Task,    // Funzione task
    "TPS2121Task",  // Nome del task
    2048,           // Dimensione dello stack
    this,           // Parametro: puntatore all'istanza corrente
    1,              // Priorità del task
    NULL            // Handle del task (non utilizzato)
  );
}
