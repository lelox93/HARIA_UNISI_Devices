#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>
#include "LedBlinker.h"

// Create an instance of the LedBlinker class
LedBlinker ledBlinker(LED_BUILTIN, 50, 2500);  // LED blinks for 200ms every 5 seconds


// BMS-101 DMG
int min_serv_dmg = 2500;
int max_serv_dmg = 600;

// Reely S-0307
int min_serv_reely = 2500;
int max_serv_reely = 600 * 2;
// int max_serv_reely = 600;

Servo servo_arm;     // create servo object to control a servo
Servo servo_finger;  // create servo object to control a servo

int servo_finger_pin = 12;
int servo_arm_pin = 14;

int min_vibro_val = 0;
int max_vibro_val = 1024;

int vibro_finger = 16;  // pin which controls the vibromotor on the finger
// int vibro_finger = 10;  // pin which controls the vibromotor on the finger
int vibro_arm = 15;  // pin which controls the vibromotor on the arm

// Define button pins
const int buttonPin1 = 4;   // Change as necessary
const int buttonPin2 = 5;   // Change as necessary
const int buttonPin3 = 13;  // Change as necessary

unsigned long lastRiseTime1 = 0;
unsigned long lastRiseTime2 = 0;
unsigned long lastRiseTime3 = 0;

const unsigned long validPressDuration = 100;  // 50ms to consider a valid button press

// Function prototypes for ISRs
void buttonPin1_isr();
void buttonPin2_isr();
void buttonPin3_isr();

uint8_t macMaster[] = { 0xE4, 0xB3, 0x23, 0xF1, 0x3B, 0x0C };

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

void onDataReceive(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&feedbackData, incomingData, sizeof(feedbackData));

  servo_arm.write(map(feedbackData.armPressure, 0, 255, min_serv_reely, max_serv_reely));
  servo_finger.write(map(feedbackData.fingerPressure, 0, 255, min_serv_reely, max_serv_reely));
  // servo_finger.write(map(feedbackData.fingerPressure, 0, 255, min_serv_dmg, max_serv_dmg));
  analogWrite(vibro_finger, map(feedbackData.fingerVibration, 0, 255, min_vibro_val, max_vibro_val));
  analogWrite(vibro_arm, map(feedbackData.armVibration, 0, 255, min_vibro_val, max_vibro_val));

  Serial.print(feedbackData.fingerPressure);
  Serial.print(" ");
  Serial.print(feedbackData.fingerVibration);
  Serial.print(" ");
  Serial.print(feedbackData.armVibration);
  Serial.print(" ");
  Serial.print(feedbackData.armPressure);
  Serial.println(" ");
  ESP.wdtFeed();
}

// callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
  bool printSenFlag = false;

  if (printSenFlag) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == 0 ? "Delivery Success" : "Delivery Fail");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  // servo_finger.attach(servo_finger_pin, max_serv_dmg, min_serv_dmg);
  servo_finger.attach(servo_finger_pin, max_serv_reely, min_serv_reely);
  servo_arm.attach(servo_arm_pin, max_serv_reely, min_serv_reely);
  pinMode(vibro_finger, OUTPUT);
  pinMode(vibro_arm, OUTPUT);
  digitalWrite(vibro_finger, LOW);
  digitalWrite(vibro_arm, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  ledBlinker.begin();  // Initialize the LED



  // Set button pins as input
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);

  // Attach interrupts with rising and falling edge trigger to buttons
  attachInterrupt(digitalPinToInterrupt(buttonPin1), buttonPin1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), buttonPin2_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin3), buttonPin3_isr, CHANGE);

  int pos;

  //SERVO FINGER SWEEP_____________________________________________________________________

  for (pos = min_serv_reely; pos >= max_serv_reely; pos -= 1) {  // goes from 0 degrees to 180 degrees
    servo_finger.writeMicroseconds(pos);
    delay(1);  // waits 15ms for the servo to reach the position
  }
  for (pos = max_serv_reely; pos <= min_serv_reely; pos += 1) {  // goes from 180 degrees to 0 degrees
    servo_finger.writeMicroseconds(pos);
    delay(1);  // waits 15ms for the servo to reach the position
  }

  delay(500);

  //SERVO ARM SWEEP_____________________________________________________________________

  for (pos = min_serv_reely; pos >= max_serv_reely; pos -= 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo_arm.writeMicroseconds(pos);
    delay(1);  // waits 15ms for the servo to reach the position
  }
  for (pos = max_serv_reely; pos <= min_serv_reely; pos += 1) {  // goes from 180 degrees to 0 degrees
    servo_arm.writeMicroseconds(pos);
    delay(1);  // waits 15ms for the servo to reach the position
  }
  delay(500);

  //VIBRO FINGER SWEEP_____________________________________________________________________
  for (int pos = min_vibro_val; pos <= max_vibro_val; pos += 1) {  // goes from 0 degrees to 180 degrees
    analogWrite(vibro_finger, pos);
    delay(1);
  }
  for (int pos = max_vibro_val; pos >= min_vibro_val; pos -= 1) {  // goes from 0 degrees to 180 degrees
    analogWrite(vibro_finger, pos);
    delay(1);
  }
  delay(500);

  //VIBRO ARM SWEEP_____________________________________________________________________
  for (int pos = min_vibro_val; pos <= max_vibro_val; pos += 1) {  // goes from 0 degrees to 180 degrees
    analogWrite(vibro_arm, pos);
    delay(1);
  }
  for (int pos = max_vibro_val; pos >= min_vibro_val; pos -= 1) {  // goes from 0 degrees to 180 degrees
    analogWrite(vibro_arm, pos);
    delay(1);
  }
  delay(500);



  WiFi.disconnect();
  ESP.eraseConfig();

  // Wifi STA Mode
  WiFi.mode(WIFI_STA);

  // Get Mac Add
  Serial.println();
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("\nESP-Now Receiver");

  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(macMaster, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceive);
  esp_now_register_send_cb(OnDataSent);

  delay(50);
}

void loop() {
  ledBlinker.update();             // Continuously update the LED state
  ledBlinker.autoTriggerUpdate();  // Automatically trigger blinks
  // Main loop, nothing to do here
}



// Variables to track last change and button state
volatile bool lastButtonState1 = LOW;
volatile unsigned long lastChangeTime1 = 0;

// ISR for buttonPin1
void ICACHE_RAM_ATTR buttonPin1_isr() {
  // Read the current button state
  bool buttonState = digitalRead(buttonPin1);
  unsigned long currentTime = millis();

  // Check if enough time has passed since the last change
  if (currentTime - lastChangeTime1 >= validPressDuration) {
    // Check if the button state has changed
    if (buttonState != lastButtonState1) {
      lastChangeTime1 = currentTime;
      lastButtonState1 = buttonState;

      // Determine the edge type ('P' for pressed, 'R' for released)
      char edgeType = buttonState ? 'P' : 'R';

      // Formulate the message
      buttonInfo_msg.button_number = 1;
      buttonInfo_msg.edge = edgeType;
      char message[10];
      snprintf(message, sizeof(message), "%d: %c\n", buttonInfo_msg.button_number, buttonInfo_msg.edge);
      Serial.print(message);

      // Send the message via ESP-NOW
      esp_now_send(macMaster, (uint8_t *)&buttonInfo_msg, sizeof(buttonInfo_msg));
    }
  }
}


// Variables to track last change and button state
volatile bool lastButtonState2 = LOW;
volatile unsigned long lastChangeTime2 = 0;

// ISR for buttonPin2
void ICACHE_RAM_ATTR buttonPin2_isr() {
  // Read the current button state
  bool buttonState = digitalRead(buttonPin2);
  unsigned long currentTime = millis();

  // Check if enough time has passed since the last change
  if (currentTime - lastChangeTime2 >= validPressDuration) {
    // Check if the button state has changed
    if (buttonState != lastButtonState2) {
      lastChangeTime2 = currentTime;
      lastButtonState2 = buttonState;

      // Determine the edge type ('P' for pressed, 'R' for released)
      char edgeType = buttonState ? 'P' : 'R';

      // Formulate the message
      buttonInfo_msg.button_number = 2;
      buttonInfo_msg.edge = edgeType;
      char message[10];
      snprintf(message, sizeof(message), "%d: %c\n", buttonInfo_msg.button_number, buttonInfo_msg.edge);
      Serial.print(message);

      // Send the message via ESP-NOW
      esp_now_send(macMaster, (uint8_t *)&buttonInfo_msg, sizeof(buttonInfo_msg));
    }
  }
}

// Variables to track last change and button state
volatile bool lastButtonState3 = LOW;
volatile unsigned long lastChangeTime3 = 0;

// ISR for buttonPin3
void ICACHE_RAM_ATTR buttonPin3_isr() {
  // Read the current button state
  bool buttonState = digitalRead(buttonPin3);
  unsigned long currentTime = millis();

  // Check if enough time has passed since the last change
  if (currentTime - lastChangeTime3 >= validPressDuration) {
    // Check if the button state has changed
    if (buttonState != lastButtonState3) {
      lastChangeTime3 = currentTime;
      lastButtonState3 = buttonState;

      // Determine the edge type ('P' for pressed, 'R' for released)
      char edgeType = buttonState ? 'P' : 'R';

      // Formulate the message
      buttonInfo_msg.button_number = 3;
      buttonInfo_msg.edge = edgeType;
      char message[10];
      snprintf(message, sizeof(message), "%d: %c\n", buttonInfo_msg.button_number, buttonInfo_msg.edge);
      Serial.print(message);

      // Send the message via ESP-NOW
      esp_now_send(macMaster, (uint8_t *)&buttonInfo_msg, sizeof(buttonInfo_msg));
    }
  }
}
