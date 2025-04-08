#include "Arduino.h"
#include "WiFi.h"
#include "pin_config.h"
#include "haria_logo.h"
#include <esp_now.h>

/* external library */
/* To use Arduino, you need to place lv_conf.h in the \Arduino\libraries directory */
#include "TFT_eSPI.h"  // https://github.com/Bodmer/TFT_eSPI


TFT_eSPI tft = TFT_eSPI();


// ESP8266 Mac address (first peer)
uint8_t mac_peer1[] = { 0x08, 0xF9, 0xE0, 0x7A, 0x02, 0xD5 };



// esp now protocol peers definition
esp_now_peer_info_t peer1;

// Define a struct named 'FeedbackData'
struct FeedbackData {
  uint16_t fingerPressure = 0;
  uint16_t armPressure = 0;
  uint16_t fingerVibration = 0;
  uint16_t armVibration = 0;
};

// Create an instance of the 'FeedbackData' struct named 'feedbackData'
FeedbackData feedbackData;




// Structure to hold button information
typedef struct {
  uint8_t button_number;
  char edge;
} buttonInfo;


// Create an instance of the structure
buttonInfo buttonInfo_msg;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  bool printSenFlag = false;

  if (printSenFlag) {
    Serial.print("Last Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

void onDataReceiver(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&buttonInfo_msg, incomingData, sizeof(buttonInfo_msg));
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  bool printRecFlag = false;

  if (printRecFlag) {
    Serial.print("Message received from: ");
    Serial.print(macStr);
    Serial.print(" ");
  }

char serial_write_start_byte = '$'; // Definisco una variabile per il carattere '$'
char message[5]; // Definisco un array di caratteri per il messaggio

// Formatto il messaggio con il numero del pulsante e il tipo di edge, usando serial_write_start_byte
snprintf(message, sizeof(message), "%c%d%c\n", serial_write_start_byte, buttonInfo_msg.button_number, buttonInfo_msg.edge);

// Stampo il messaggio sulla seriale
Serial.print(message);

}



int start_byte = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(TFT_LEDA_PIN, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Get Mac Add
  Serial.println();
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW Initialized!");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceiver);

  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = 1;
  peer1.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 1");
  if (esp_now_add_peer(&peer1) == ESP_OK) {
    Serial.println("Peer 1 added");
  }

  // Initialise TFT
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_DARKGREY);
  digitalWrite(TFT_LEDA_PIN, 0);
  tft.setTextFont(1);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);

  // Draw HARIA logo
  tft.pushImage(0, 0, 160, 80, (uint16_t *)gHARIA_logo);
  tft.drawString("SMI", 120, 0, 2);
  // tft.drawString("v1", 120, 13, 2);
}

void loop() {

  if (Serial.available() > 15)  //  wait for 16 bytes
  {
    start_byte = (byte)(Serial.read());
    if (start_byte == 36)  //  36 is the dec for '$' ASCII character - START
    {
      feedbackData.fingerPressure = Serial.parseInt();
      feedbackData.armPressure = Serial.parseInt();
      feedbackData.fingerVibration = Serial.parseInt();
      feedbackData.armVibration = Serial.parseInt();

      esp_now_send(peer1.peer_addr, (uint8_t *)&feedbackData, sizeof(feedbackData));
    }
  }
}