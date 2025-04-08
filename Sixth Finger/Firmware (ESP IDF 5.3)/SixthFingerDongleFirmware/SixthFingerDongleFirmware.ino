#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "pin_config.h"        // Definisce ad es. TFT_LEDA_PIN
#include "haria_logo.h"        // Immagine del logo
#include "SixthFingerTypes.h"  // Definizioni delle strutture (SixthFingerCMD, SixthFingerData, ecc.)
#include <TFT_eSPI.h>          // Libreria per il display TFT

// Istanza del display TFT
TFT_eSPI tft = TFT_eSPI();

// Indirizzo MAC del peer ESP‑NOW
uint8_t mac_peer1[] = { 0xC8, 0x2E, 0x18, 0xB3, 0x48, 0x18 };
esp_now_peer_info_t peer1;

// Strutture per i messaggi (definite in SixthFingerTypes.h)
SixthFingerCMD   sixthFingerCMD;
SixthFingerData  sixthFingerData;
SixthFingerPos   sixthFingerPos;

// Variabili di stato per il monitoraggio di invio/ricezione
volatile bool check_sent = true;
volatile bool check_sent_prec = false;
volatile bool check_received = true;
volatile bool check_received_prec = false;
volatile int  check_received_counter = 0;
char sf_data_str_buf[20] = {0};
int start_byte = 0;

// Coda per i messaggi da inviare e handle per il task di invio
QueueHandle_t sendQueue;
TaskHandle_t sendTaskHandle;

// Struttura del pacchetto dati per ESP‑NOW
struct DataPacket {
  uint8_t mac_addr[6];
  uint8_t *data;
  size_t len;
};

// Funzione di utilità: converte un intero in stringa a 3 cifre
String intToStringThreeDigits(int val) {
  char formattedString[4];  // 3 cifre + terminatore
  sprintf(formattedString, "%03d", val);
  return String(formattedString);
}

// *** Callback aggiornata per ESP‑NOW ***
// La firma ora deve essere: (const esp_now_recv_info_t*, const uint8_t*, int)
void onDataReceiver(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Se necessario, puoi ottenere il MAC del mittente tramite:
  // const uint8_t *mac = recv_info->src_addr;
  memcpy(&sixthFingerData, incomingData, sizeof(sixthFingerData));

  // Aggiorna la stringa formattata: "$<load> <pos> <C/O>\n"
  snprintf(sf_data_str_buf, sizeof(sf_data_str_buf), "$%s %s %c\n",
           intToStringThreeDigits(sixthFingerData.motor1.load).c_str(),
           intToStringThreeDigits(sixthFingerData.motor1.positionPercentage).c_str(),
           (sixthFingerData.status == SixthFingerStatus::CLOSED) ? 'C' : 'O');

  check_received = true;
  check_received_counter++;
}

// Funzione per inserire il messaggio SixthFingerCMD nella coda
void enqueueSixthFingerCMD() {
  DataPacket packet;
  memcpy(packet.mac_addr, peer1.peer_addr, 6);
  packet.data = (uint8_t *)malloc(sizeof(sixthFingerCMD));
  memcpy(packet.data, &sixthFingerCMD, sizeof(sixthFingerCMD));
  packet.len = sizeof(sixthFingerCMD);
  if (xQueueSend(sendQueue, &packet, portMAX_DELAY) != pdTRUE) {
    free(packet.data);
  }
}

// Funzione per inserire il messaggio SixthFingerPos nella coda
void enqueueSixthFingerPos() {
  DataPacket packet;
  memcpy(packet.mac_addr, peer1.peer_addr, 6);
  packet.data = (uint8_t *)malloc(sizeof(sixthFingerPos));
  memcpy(packet.data, &sixthFingerPos, sizeof(sixthFingerPos));
  packet.len = sizeof(sixthFingerPos);
  if (xQueueSend(sendQueue, &packet, portMAX_DELAY) != pdTRUE) {
    free(packet.data);
  }
}

// Task per l'invio dei dati via ESP‑NOW
void sendTask(void *pvParameter) {
  DataPacket packet;
  for (;;) {
    if (xQueueReceive(sendQueue, &packet, portMAX_DELAY) == pdTRUE) {
      esp_err_t result = esp_now_send(packet.mac_addr, packet.data, packet.len);
      if (result == ESP_OK) {
        check_sent = true;
      }
      free(packet.data);
    }
    vTaskDelay(pdMS_TO_TICKS(67));  // Frequenza di circa 15 Hz
  }
}

// Funzioni grafiche per disegnare le frecce dei dati sul display TFT
void drawOutDataArrow(bool en) {
  if (en) {
    tft.fillRect(120, 62, 30, 3, TFT_DARKGREEN);
    tft.fillTriangle(115 + 30, 58, 115 + 30, 64, 115 + 30 + 7, 64, TFT_DARKGREEN);
  } else {
    tft.fillRect(120, 62, 30, 3, TFT_WHITE);
    tft.fillTriangle(115 + 30, 58, 115 + 30, 64, 115 + 30 + 7, 64, TFT_WHITE);
  }
}

void drawInDataArrow(bool en) {
  tft.fillRect(123, 67, 30, 3, en ? TFT_DARKGREEN : TFT_WHITE);
  tft.fillTriangle(128, 74, 128, 67, 121, 67, en ? TFT_DARKGREEN : TFT_WHITE);
}

// Task per aggiornare lo stato grafico (al posto del timer da 500 ms)
void updateTask(void *pvParameter) {
  for (;;) {
    // Gestione dell'arrow per la ricezione
    if (check_received) {
      if (!check_received_prec) {
        drawInDataArrow(true);
      }
      if (check_received_counter == 0) {
        check_received_prec = check_received;
        check_received = false;
      }
    } else {
      if (!check_received_prec) {
        drawInDataArrow(false);
      }
      check_received_prec = check_received;
      check_received = false;
    }

    // Gestione dell'arrow per l'invio
    if (check_sent) {
      if (!check_sent_prec) {
        drawOutDataArrow(true);
      }
      check_sent_prec = check_sent;
      check_sent = false;
    } else {
      if (!check_sent_prec) {
        drawOutDataArrow(false);
      }
      check_sent_prec = check_sent;
      check_sent = false;
    }

    check_received_counter = 0;
    vTaskDelay(pdMS_TO_TICKS(500)); // Aggiornamento ogni 500 ms
  }
}

// Task per la stampa su seriale (al posto del timer da 30 ms)
void printTask(void *pvParameter) {
  for (;;) {
    // Se sono stati ricevuti dati, stampa la stringa formattata
    if (check_received) {
      Serial.print(sf_data_str_buf);
    }
    vTaskDelay(pdMS_TO_TICKS(30));  // Attesa di 30 ms tra le stampe
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Attesa per la stabilizzazione della seriale

  pinMode(TFT_LEDA_PIN, OUTPUT);

  // Imposta la modalità WiFi in STA (Station)
  WiFi.mode(WIFI_STA);
  
  // Inizializza ESP‑NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW Initialized!");
  }

  // Stampa il MAC address
  Serial.println();
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());

  // Registra la callback per la ricezione dei dati via ESP‑NOW
  esp_now_register_recv_cb(onDataReceiver);

  // Configura il peer ESP‑NOW
  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = 1;
  peer1.encrypt = false;
  Serial.println("Registering peer 1");
  if (esp_now_add_peer(&peer1) == ESP_OK) {
    Serial.println("Peer 1 added");
  } else {
    Serial.println("Error adding peer 1");
  }

  // Inizializza il display TFT
  tft.init();
  tft.setRotation(5);
  tft.setViewport(1, 26, 160, 80, true);
  tft.setOrigin(1, 26);
  tft.fillScreen(TFT_DARKGREY);
  delay(1000);
  tft.fillScreen(TFT_WHITE);
  delay(1000);
  digitalWrite(TFT_LEDA_PIN, LOW);
  tft.setTextFont(1);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.pushImage(0, 0, 160, 80, (uint16_t *)gHARIA_logo);
  tft.drawString("Sixth", 120, 0, 2);
  tft.drawString("Finger_v1", 120, 13, 2);
  drawInDataArrow(true);

  // Crea la coda per i pacchetti da inviare
  sendQueue = xQueueCreate(10, sizeof(DataPacket));

  // Crea il task per l'invio dei dati via ESP‑NOW
  xTaskCreate(sendTask, "SendTask", 4096, NULL, 1, &sendTaskHandle);

  // Crea i task per l'aggiornamento grafico e la stampa su seriale
  xTaskCreate(updateTask, "UpdateTask", 2048, NULL, 1, NULL);
  xTaskCreate(printTask, "PrintTask", 2048, NULL, 1, NULL);
}

void loop() {
  // Gestione dei dati ricevuti dalla seriale
  if (Serial.available() > 5) {
    start_byte = Serial.read();
    if (start_byte == '$') {
      char first_byte = Serial.read();
      if (first_byte == 'O' || first_byte == 'C' || first_byte == 'S' || first_byte == 'V') {
        char second_byte = Serial.read();
        sixthFingerCMD.cmd = first_byte;
        sixthFingerCMD.aux = second_byte;
        enqueueSixthFingerCMD();
        // Scarta i byte in eccesso (i restanti 3 byte)
        for (int i = 0; i < 3; i++) {
          Serial.read();
        }
      } else if (first_byte == 'N') {
        // Il comando "N" viene seguito da un numero (fine comando con newline)
        sixthFingerPos.motor1_pos = Serial.parseInt();
        enqueueSixthFingerPos();
      }
    }
    // Puoi gestire ulteriori comandi qui se necessario.
  }
  // Il loop principale resta libero; le attività periodiche sono gestite dai task.
}
