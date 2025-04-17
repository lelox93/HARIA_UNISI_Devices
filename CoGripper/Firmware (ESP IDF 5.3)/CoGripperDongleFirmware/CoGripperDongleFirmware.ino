#include "Arduino.h"
#include "WiFi.h"
#include "pin_config.h"
#include "haria_logo.h"
#include "CoGripperTypes.h"
#include <esp_now.h>
#include "TFT_eSPI.h"  // https://github.com/Bodmer/TFT_eSPI

// Istanza del display TFT
TFT_eSPI tft = TFT_eSPI();

// Indirizzo MAC del peer ESP‑NOW
uint8_t mac_peer1[] = { 0xA0, 0xA3, 0xB3, 0x2A, 0xE6, 0x60 };

// Struttura del peer ESP‑NOW
esp_now_peer_info_t peer1;

// Strutture per i messaggi
CoGripperCMD  coGripperCMD;
CoGripperData coGripperData;
CoGripperPos  coGripperPos;

// Variabili di stato per monitoraggio invio/ricezione
volatile bool check_sent = true;
volatile bool check_sent_prec = false;
volatile bool check_received = true;
volatile bool check_received_prec = false;
volatile int  check_received_counter = 0;

// Buffer per la stringa formattata da stampare su Serial
char sf_data_str_buf[20];

// Variabile per il parsing dei comandi via Serial
int start_byte = 0;

// Coda per i pacchetti da inviare e handle per il task di invio
QueueHandle_t sendQueue;
TaskHandle_t sendTaskHandle;

// Prototipi dei task aggiuntivi
void updateTask(void *pvParameter);
void printTask(void *pvParameter);

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

// Callback per la ricezione dei dati via ESP‑NOW (firma aggiornata per ESP‑IDF 5.3)
void onDataReceiver(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Se necessario, l'indirizzo MAC del mittente può essere ottenuto tramite:
  // const uint8_t *mac = recv_info->src_addr;
  memcpy(&coGripperData, incomingData, sizeof(coGripperData));

  // Aggiorna la stringa formattata: "$<m1 load> <m1 %> <m2 load> <m2 %> <C/O>\n"
  snprintf(sf_data_str_buf, sizeof(sf_data_str_buf), "$%s %s %s %s %c\n",
           intToStringThreeDigits(coGripperData.motor1.load).c_str(),
           intToStringThreeDigits(coGripperData.motor1.positionPercentage).c_str(),
           intToStringThreeDigits(coGripperData.motor2.load).c_str(),
           intToStringThreeDigits(coGripperData.motor2.positionPercentage).c_str(),
           coGripperData.status == CoGripperStatus::CLOSED ? 'C' : 'O');
  check_received = true;
  check_received_counter++;  // Incrementa il contatore di ricezione
}

// Funzione per inserire il messaggio CoGripperCMD nella coda
void enqueueCoGripperCMD() {
  DataPacket packet;
  memcpy(packet.mac_addr, peer1.peer_addr, 6);
  packet.data = (uint8_t *)malloc(sizeof(coGripperCMD));
  memcpy(packet.data, &coGripperCMD, sizeof(coGripperCMD));
  packet.len = sizeof(coGripperCMD);
  if (xQueueSend(sendQueue, &packet, portMAX_DELAY) != pdTRUE) {
    free(packet.data);
  }
}

// Funzione per inserire il messaggio CoGripperPos nella coda
void enqueueCoGripperPos() {
  DataPacket packet;
  memcpy(packet.mac_addr, peer1.peer_addr, 6);
  packet.data = (uint8_t *)malloc(sizeof(coGripperPos));
  memcpy(packet.data, &coGripperPos, sizeof(coGripperPos));
  packet.len = sizeof(coGripperPos);
  if (xQueueSend(sendQueue, &packet, portMAX_DELAY) != pdTRUE) {
    free(packet.data);
  }
}

// Task per l'invio dei dati via ESP‑NOW (circa 15 Hz)
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
    vTaskDelay(pdMS_TO_TICKS(67));  // Delay per limitare la frequenza a circa 15 Hz
  }
}

// Task per aggiornare graficamente lo stato ogni 500 ms (simile a onTimer)
void updateTask(void *pvParameter) {
  for (;;) {
    // Gestione freccia dati in ingresso
    if (check_received) {
      if (!check_received_prec) {
        // Abilita freccia ingresso
        tft.fillRect(123, 67, 30, 3, TFT_DARKGREEN);
        tft.fillTriangle(128, 74, 128, 67, 121, 67, TFT_DARKGREEN);
      }
      if (check_received_counter == 0) {
        check_received_prec = check_received;
        check_received = false;
      }
    } else {
      if (!check_received_prec) {
        // Disabilita freccia ingresso
        tft.fillRect(123, 67, 30, 3, TFT_WHITE);
        tft.fillTriangle(128, 74, 128, 67, 121, 67, TFT_WHITE);
      }
      check_received_prec = check_received;
      check_received = false;
    }

    // Gestione freccia dati in uscita
    if (check_sent) {
      if (!check_sent_prec) {
        // Abilita freccia uscita
        tft.fillRect(120, 62, 30, 3, TFT_DARKGREEN);
        tft.fillTriangle(145, 58, 145, 64, 152, 64, TFT_DARKGREEN);
      }
      check_sent_prec = check_sent;
      check_sent = false;
    } else {
      if (!check_sent_prec) {
        // Disabilita freccia uscita
        tft.fillRect(120, 62, 30, 3, TFT_WHITE);
        tft.fillTriangle(145, 58, 145, 64, 152, 64, TFT_WHITE);
      }
      check_sent_prec = check_sent;
      check_sent = false;
    }

    check_received_counter = 0;
    vTaskDelay(pdMS_TO_TICKS(500)); // Aggiornamento ogni 500 ms
  }
}

// Task per stampare su Serial ogni 30 ms (simile a printTask)
void printTask(void *pvParameter) {
  for (;;) {
    if (check_received) {
      Serial.print(sf_data_str_buf);
    }
    vTaskDelay(pdMS_TO_TICKS(30));  // Delay di 30 ms
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(TFT_LEDA_PIN, OUTPUT);

  // Imposta il dispositivo in modalità Wi-Fi Station
  WiFi.mode(WIFI_STA);
  delay(2000);
  bool verbose_init = true;
  if (verbose_init) {
    Serial.println();
    Serial.print("Mac Address: ");
    Serial.println(WiFi.macAddress());
  }

  // Inizializza ESP‑NOW
  if (esp_now_init() != ESP_OK) {
    if (verbose_init) {
      Serial.println("Error initializing ESP-NOW");
    }
    return;
  } else {
    if (verbose_init) {
      Serial.println("ESP-NOW Initialized!");
    }
  }

  // Registra la callback per la ricezione dei dati via ESP‑NOW (nuova firma)
  esp_now_register_recv_cb(onDataReceiver);

  // Configura il peer ESP‑NOW
  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = 1;
  peer1.encrypt = false;
  if (verbose_init) {
    Serial.println("Registering peer 1");
  }
  if (esp_now_add_peer(&peer1) == ESP_OK) {
    if (verbose_init) {
      Serial.println("Peer 1 added");
    }
  }

  // Inizializza il display TFT e applica le nuove impostazioni (setRotation, setViewport, setOrigin)
  tft.init();
  tft.setRotation(5);
  tft.setViewport(1, 26, 160, 80, true);
  tft.setOrigin(1, 26);
  tft.fillScreen(TFT_DARKGREY);
  digitalWrite(TFT_LEDA_PIN, LOW);
  tft.setTextFont(1);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);

  // Visualizza il logo HARIA
  tft.pushImage(0, 0, 160, 80, (uint16_t *)gHARIA_logo);
  tft.drawString("CoGrip", 120, 0, 2);
  tft.drawString("v1", 120, 13, 2);
  
  // Disegna la freccia dati in ingresso inizialmente attiva
  tft.fillRect(123, 67, 30, 3, TFT_DARKGREEN);
  tft.fillTriangle(128, 74, 128, 67, 121, 67, TFT_DARKGREEN);

  // Crea la coda per i pacchetti da inviare
  sendQueue = xQueueCreate(10, sizeof(DataPacket));

  // Crea i task FreeRTOS
  xTaskCreate(sendTask, "SendTask", 4096, NULL, 1, &sendTaskHandle);
  xTaskCreate(updateTask, "UpdateTask", 2048, NULL, 1, NULL);
  xTaskCreate(printTask, "PrintTask", 2048, NULL, 1, NULL);
}

void loop() {
  // Gestione dei comandi via Serial
  int bytes_available = Serial.available();
  if (bytes_available > 11) {  // Attende almeno 12 byte
    start_byte = Serial.read();
    if (start_byte == '$') {  // Carattere di inizio comando
      char first_byte = Serial.read();
      if (first_byte == 'O' || first_byte == 'C' || first_byte == 'S' || first_byte == 'V') {
        char second_byte = Serial.read();
        coGripperCMD.cmd = first_byte;
        coGripperCMD.aux = second_byte;
        enqueueCoGripperCMD();
        // Scarta i byte in eccesso (8 byte)
        for (int i = 0; i < 8; i++) {
          Serial.read();
        }
      } else if (first_byte == 'N') {
        // Comando per impostare le posizioni di due motori
        coGripperPos.motor1_pos = Serial.parseInt();
        coGripperPos.motor2_pos = Serial.parseInt();
        enqueueCoGripperPos();
      }
    }
  }
  // Il loop resta libero: le attività periodiche sono gestite dai task FreeRTOS
}
