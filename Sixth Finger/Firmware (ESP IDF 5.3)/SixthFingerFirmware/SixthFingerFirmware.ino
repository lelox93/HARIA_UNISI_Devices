#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>
#include "TM2011.h"
#include "SixthFingerController.h"
#include "EspNowManager.h"  // Classe che gestisce ESP-NOW

#include <WiFi.h>
#include <esp_now.h>

#include "RgbLedManager.h"

//==============================================================================
// Definizioni dei pin e istanze globali
//==============================================================================

#define DYNA_TX_PIN 17       ///< Pin TX per la comunicazione seriale con i Dynamixel
#define DYNA_RX_PIN 16       ///< Pin RX per la comunicazione seriale con i Dynamixel
const int DXL_DIR_PIN = 21;  ///< Pin di direzione per il DYNAMIXEL Shield

SixthFingerController controller(DYNA_TX_PIN, DYNA_RX_PIN, DXL_DIR_PIN);  ///< Controller del sistema

// Esempio: gestione di un singolo LED
#define NEOPIXEL_PIN 4
#define NUMPIXELS 1
RgbLedManager ledManager(NUMPIXELS, NEOPIXEL_PIN);


TM2011 buttonController;  ///< Gestione dei pulsanti interni/esterni

/**
 * @brief Struttura per la gestione dei comandi ricevuti tramite ESP‑NOW.
 */
struct SixthFingerCMD {
  char cmd = 'O';  ///< Comando principale: 'C' (close), 'O' (open), 'S' (stop), 'V' (velocità)
  char aux = 'O';  ///< Comando ausiliario: per 'V': 'S' (slow) oppure 'F' (fast)
};

/**
 * @brief Struttura per aggiornare la posizione del motore 1.
 */
struct SixthFingerPos {
  int motor1_pos;  ///< Posizione richiesta per il motore 1
};

SixthFingerCMD sixthfingerCMD;  ///< Variabile per il comando ricevuto
SixthFingerPos sixthfingerPos;  ///< Variabile per la posizione ricevuta

//==============================================================================
// Callback e funzioni di utilità per ESP‑NOW
//==============================================================================

/**
 * @brief Callback chiamata dopo l'invio di un pacchetto ESP‑NOW.
 *
 * @param mac_addr Indirizzo MAC del destinatario.
 * @param status Stato dell'invio.
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Se abilitato, stampa lo stato dell'invio
  bool printSenFlag = false;
  if (printSenFlag) {
    Serial.print("Last Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

/**
 * @brief Callback per la ricezione dei dati tramite ESP‑NOW.
 *
 * Verifica la lunghezza del pacchetto e aggiorna la struttura corrispondente.
 *
 * @param recv_info Informazioni sul ricevitore.
 * @param incomingData Puntatore ai dati ricevuti.
 * @param len Lunghezza dei dati ricevuti.
 */
void onDataReceiver(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(SixthFingerCMD)) {
    memcpy(&sixthfingerCMD, incomingData, len);
  } else if (len == sizeof(SixthFingerPos)) {
    memcpy(&sixthfingerPos, incomingData, len);
    // Limita e mappa il valore ricevuto
    sixthfingerPos.motor1_pos = intConstrain(sixthfingerPos.motor1_pos, 0, 4200);
    controller.setGoalPositionMotor1(
      map(sixthfingerPos.motor1_pos, 0, 4200,
          controller.getMotor1EndPositionMin(), controller.getMotor1EndPositionMax()));
  } else {
    Serial.println("Messaggio di lunghezza non supportata ricevuto");
  }
}

/**
 * @brief Limita un intero all'intervallo [min, max].
 *
 * @param value Valore da limitare.
 * @param min Valore minimo.
 * @param max Valore massimo.
 * @return int Valore limitato.
 */
int intConstrain(int value, int min, int max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/**
 * @brief Esegue il comando ricevuto tramite ESP‑NOW.
 *
 * Interpreta il comando e attiva l'azione corrispondente sul controller.
 */
void serviceCMD() {
  switch (sixthfingerCMD.cmd) {
    case 'C':
      Serial.println("CLOSE ALL");
      controller.closeAllMotors();
      break;
    case 'O':
      Serial.println("OPEN ALL");
      controller.openAllMotors();
      break;
    case 'S':
      Serial.println("STOP ALL");
      controller.stopAllMotors();
      break;
    case 'V':
      if (sixthfingerCMD.aux == 'S') {
        Serial.println("SLOW");
        controller.motor1.configureVelocity(LOW);
      } else if (sixthfingerCMD.aux == 'F') {
        Serial.println("FAST");
        controller.motor1.configureVelocity(HIGH);
      }
      break;
    default:
      Serial.println("Comando non riconosciuto");
      break;
  }
}

//==============================================================================
// Integrazione di ESP‑NOW tramite EspNowManager
//==============================================================================

// Indirizzo MAC del peer
uint8_t mac_peer1[] = { 0xDC, 0xDA, 0x0C, 0x30, 0xC2, 0x74 };
EspNowManager espManager(mac_peer1);  ///< Istanza della classe EspNowManager

//==============================================================================
// Task FreeRTOS per il controllo del sistema
//==============================================================================

/**
 * @brief Task che gestisce la logica di controllo.
 *
 * Legge gli stati dei pulsanti, processa eventuali comandi ricevuti tramite ESP‑NOW
 * ed esegue il ciclo di controllo del controller.
 *
 * Questo task viene eseguito in loop a intervalli regolari.
 *
 * @param parameter Parametro del task (non utilizzato).
 */
void controlTask(void *parameter) {
  for (;;) {
    // Gestione dei pulsanti interni ed esterni
    ButtonState internalState = buttonController.getInternalButtonState(false);
    controller.handleButtonStates(internalState);

    ButtonState externalState = buttonController.getExternalButtonState(false);
    controller.handleExtButtonStates(externalState);

    // Se è stato ricevuto un comando, esegui l'azione corrispondente
    if (sixthfingerCMD.cmd != 'O') {
      serviceCMD();
      // Resetta il comando a 'O' (stato neutro)
      sixthfingerCMD.cmd = 'O';
    }

    // Esegue il ciclo di controllo del controller
    controller.controlLoop();

    // Delay breve per consentire la concorrenza degli altri task
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

//==============================================================================
// Setup e Loop
//==============================================================================

/**
 * @brief Funzione di inizializzazione.
 *
 * Configura Serial, LED di sistema, ESP‑NOW tramite EspNowManager, la striscia NeoPixel
 * e il controller del sistema.
 */
void setup() {
  Serial.begin(57600);
  delay(2000);


  // Inizializza eventuali effetti LED (opzionale)
  ledManager.begin();
  ledManager.fadeToColor(100, 100, 0, 1000);  // Esegue un fade al giallino in 1 secondo
  delay(2000);

  // Inizializza ESP‑NOW tramite EspNowManager
  Serial.println("\nUSING ESP NOW!");
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (!espManager.begin()) {
    Serial.println("Error initializing ESP-NOW");
  } else {
    Serial.println("ESP-NOW Initialized!");
    espManager.registerCallbacks(OnDataSent, onDataReceiver);
    if (espManager.addPeer()) {
      Serial.println("Peer added successfully");
    }
    espManager.startSendTask();
  }

  // Inizializza e calibra il controller e il motore
  if (controller.initialize()) {
    ledManager.fadeToColor(0, 255, 0, 1000);  // Esegue un fade al verde in 1 secondo
  } else {
    ledManager.fadeToColor(255, 0, 0, 1000);  // Esegue un fade al rosso in 1 secondo
  }

  controller.calibrateLimits();
  controller.setSixthFingerMaxPWM(375);

  // Crea il task FreeRTOS per il controllo del sistema
  xTaskCreate(controlTask, "ControlTask", 2048, NULL, 1, NULL);
  ledManager.fadeToColor(0, 255, 0, 1000);  // Esegue un fade al verde in 1 secondo
}

/**
 * @brief Loop principale.
 *
 * Visto che la logica è gestita dai task FreeRTOS, il loop rimane essenzialmente vuoto.
 */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
