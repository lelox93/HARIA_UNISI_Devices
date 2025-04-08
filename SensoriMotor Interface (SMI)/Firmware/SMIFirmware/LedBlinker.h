#ifndef LEDBLINKER_H
#define LEDBLINKER_H

#include <Arduino.h>

class LedBlinker {
private:
    int ledPin;                   // The LED pin to control
    unsigned long blinkDuration;  // Duration for which the LED stays LOW
    unsigned long autoTriggerInterval; // Interval to auto-trigger the LED blink
    unsigned long previousMillis; // Tracks the last time the LED state was changed
    unsigned long lastAutoTrigger; // Tracks the last auto-trigger time
    bool isBlinking;              // Flag to indicate if the LED is in a blink cycle

public:
    // Constructor
    LedBlinker(int pin, unsigned long duration, unsigned long autoInterval);

    // Initialize the LED pin
    void begin();

    // Trigger the blink (set LED LOW for the duration)
    void trigger();

    // Update the LED state
    void update();

    // Automatically trigger the blink every autoTriggerInterval milliseconds
    void autoTriggerUpdate();
};

#endif // LEDBLINKER_H
