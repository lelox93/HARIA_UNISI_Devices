#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>
#include "TM2011.h"
#include "CoGripperController.h"
#include "EspNowManager.h"  // Classe che gestisce ESP‑NOW
#include <WiFi.h>
#include <esp_now.h>
#include "RgbLedManager.h"

//==============================================================================
// Definizioni dei pin e istanze globali
//==============================================================================

#define DYNA_TX_PIN 17       ///< Pin TX per la comunicazione seriale con i Dynamixel
#define DYNA_RX_PIN 16       ///< Pin RX per la comunicazione seriale con i Dynamixel
const int DXL_DIR_PIN = 21;  ///< Pin di direzione per il DYNAMIXEL Shield

// Istanza del controller del gripper
CoGripperController controller(DYNA_TX_PIN, DYNA_RX_PIN, DXL_DIR_PIN);  ///< Controller del sistema

// Gestione dei pulsanti tramite TM2011 (al posto di SLLB120100)
TM2011 buttonController;

// Esempio: gestione di un singolo LED
#define NEOPIXEL_PIN 4
#define NUMPIXELS 1
RgbLedManager ledManager(NUMPIXELS, NEOPIXEL_PIN);

// ──────────────────────────────────────────────
// ESP-NOW configuration tramite EspNowManager
uint8_t mac_peer1[] = { 0xF0, 0x9E, 0x9E, 0x28, 0xA3, 0x24 };
EspNowManager espNowManager(mac_peer1, 1, false);  // channel 1, no encryption

// ──────────────────────────────────────────────
// Definizione dei comandi da ricevere via ESP‑NOW

/**
 * @brief Struttura per i comandi del gripper.
 */
struct CoGripperCMD {
  char cmd = 'O';
  char aux = 'O';
};

/**
 * @brief Struttura per i comandi di posizione.
 */
struct CoGripperPos {
  int motor1_pos;
  int motor2_pos;
};

CoGripperCMD cogripperCMD;
CoGripperPos coGripperPos;

// Flag per attivare la gestione del comando ricevuto via ESP-NOW
volatile bool service_CMD = false;

// ──────────────────────────────────────────────
// Callback per l'invio dei dati via ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Per debug, se necessario:
  // Serial.print("Packet send status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ──────────────────────────────────────────────
// Callback per la ricezione dei dati (firma aggiornata per ESP-IDF 5.3)
void onDataReceiver(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(CoGripperCMD)) {
    memcpy(&cogripperCMD, incomingData, len);
    service_CMD = true;
  } else if (len == sizeof(CoGripperPos)) {
    memcpy(&coGripperPos, incomingData, len);
    coGripperPos.motor1_pos = intConstrain(coGripperPos.motor1_pos, 0, 2400);
    coGripperPos.motor2_pos = intConstrain(coGripperPos.motor2_pos, 0, 2400);
    controller.setGoalPositionMotor1(
      map(coGripperPos.motor1_pos, 0, 2400,
          controller.getMotor1EndPositionMin(), controller.getMotor1EndPositionMax()));
    controller.setGoalPositionMotor2(
      map(coGripperPos.motor2_pos, 0, 2400,
          controller.getMotor2EndPositionMin(), controller.getMotor2EndPositionMax()));
  } else {
    Serial.println("Messaggio di lunghezza non supportata ricevuto");
  }
}

// ──────────────────────────────────────────────
// Task per l'invio periodico dei dati via ESP-NOW (ogni 30 ms)
void espNowSendTask(void *pvParameters) {
  for (;;) {
    CoGripperData dataToSend = controller.getData();
    espNowManager.sendData((uint8_t *)&dataToSend, sizeof(dataToSend));
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ──────────────────────────────────────────────
// Utility: Constrains un valore tra un minimo e un massimo.
int intConstrain(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// ──────────────────────────────────────────────
// Funzione per gestire i comandi ricevuti via ESP-NOW
void serviceCMD() {
  if (cogripperCMD.cmd == 'C') {
    if (cogripperCMD.aux == '1') {
      controller.motor1.setGoalPosition(controller.motor1.getEndPositionMin());
      Serial.println("CLOSE 1");
    } else if (cogripperCMD.aux == '2') {
      controller.motor2.setGoalPosition(controller.motor2.getEndPositionMin());
      Serial.println("CLOSE 2");
    } else {
      Serial.println("CLOSE ALL");
      controller.closeAllMotors();
    }
  } else if (cogripperCMD.cmd == 'O') {
    if (cogripperCMD.aux == '1') {
      controller.motor1.setGoalPosition(controller.motor1.getEndPositionMax());
      Serial.println("OPEN 1");
    } else if (cogripperCMD.aux == '2') {
      controller.motor2.setGoalPosition(controller.motor2.getEndPositionMax());
      Serial.println("OPEN 2");
    } else {
      Serial.println("OPEN ALL");
      controller.openAllMotors();
    }
  } else if (cogripperCMD.cmd == 'S') {
    if (cogripperCMD.aux == '1') {
      controller.motor1.setGoalPosition(controller.motor1.getCurrentPosition());
      Serial.println("STOP 1");
    } else if (cogripperCMD.aux == '2') {
      controller.motor2.setGoalPosition(controller.motor2.getCurrentPosition());
      Serial.println("STOP 2");
    } else {
      Serial.println("STOP ALL");
      controller.stopAllMotors();
    }
  } else if (cogripperCMD.cmd == 'V') {
    if (cogripperCMD.aux == 'S') {
      Serial.println("SLOW");
      controller.motor1.configureVelocity(LOW);
      controller.motor2.configureVelocity(LOW);
    }
    if (cogripperCMD.aux == 'F') {
      Serial.println("FAST");
      controller.motor1.configureVelocity(HIGH);
      controller.motor2.configureVelocity(HIGH);
    }
  }
}

// ──────────────────────────────────────────────
// setup()
void setup() {
  Serial.begin(57600);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(1000);
  // Inizializza eventuali effetti LED (opzionale)
  ledManager.begin();
  ledManager.fadeToColor(100, 100, 0, 1000);  // Esegue un fade al verde in 1 secondo
  delay(2000);
  // Inizializza WiFi e configura ESP-NOW tramite EspNowManager
  Serial.println("\nUSING ESP NOW!");
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());

  if (!espNowManager.begin()) {
    Serial.println("Error initializing ESP‑NOW");
    return;
  } else {
    Serial.println("ESP‑NOW Initialized!");
  }

  // Registra le callback di invio e ricezione tramite EspNowManager
  espNowManager.registerCallbacks(onDataSent, onDataReceiver);

  // Aggiunge il peer
  if (!espNowManager.addPeer()) {
    Serial.println("Failed to add ESP‑NOW peer");
  } else {
    Serial.println("Peer added");
  }

  // Crea il task FreeRTOS per inviare periodicamente i dati via ESP‑NOW
  xTaskCreate(espNowSendTask, "espNowSendTask", 2048, NULL, 1, NULL);

  // Inizializza il controller del gripper
  if (controller.initialize()) {
    ledManager.fadeToColor(0, 255, 0, 1000);  // Esegue un fade al verde in 1 secondo
  } else {
    ledManager.fadeToColor(255, 0, 0, 1000);  // Esegue un fade al rosso in 1 secondo
  }

  controller.calibrateLimits();
  controller.setGripperMaxPWM(375);

  // rgbLed.setStaticColor(0, 255, 0); // Esempio: LED verde statico
}

// ──────────────────────────────────────────────
// loop()
void loop() {
  // Gestione dei pulsanti interni ed esterni
  ButtonState internalState = buttonController.getInternalButtonState(false);
  controller.handleButtonStates(internalState);

  // ButtonState externalState = buttonController.getExternalButtonState(false);
  // controller.handleExtButtonStates(externalState);

  // Gestisce il comando ricevuto via ESP-NOW, se presente
  if (service_CMD) {
    serviceCMD();
    service_CMD = false;
  }

  // Esegue il ciclo di controllo del gripper
  controller.controlLoop();

  delay(2);
}
