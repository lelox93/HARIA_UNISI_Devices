#include "LedBlinker.h"

// Constructor
LedBlinker::LedBlinker(int pin, unsigned long duration, unsigned long autoInterval)
    : ledPin(pin), blinkDuration(duration), autoTriggerInterval(autoInterval),
      previousMillis(0), lastAutoTrigger(0), isBlinking(false) {}

// Initialize the LED pin
void LedBlinker::begin() {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH); // Start with the LED HIGH
}

// Trigger the blink (set LED LOW for the duration)
void LedBlinker::trigger() {
    digitalWrite(ledPin, LOW);
    previousMillis = millis(); // Record the current time
    isBlinking = true;         // Indicate that a blink is active
}

// Update the LED state
void LedBlinker::update() {
    if (isBlinking && (millis() - previousMillis >= blinkDuration)) {
        digitalWrite(ledPin, HIGH); // Turn the LED back HIGH
        isBlinking = false;         // Reset the blinking flag
    }
}

// Automatically trigger the blink every autoTriggerInterval milliseconds
void LedBlinker::autoTriggerUpdate() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastAutoTrigger >= autoTriggerInterval) {
        trigger();                   // Trigger the blink
        lastAutoTrigger = currentMillis; // Update the last trigger time
    }
}
