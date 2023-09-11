#include <Arduino.h>
#include "INTERFACE.h"

const uint8_t ball_radius = 20;
INTERFACE& interface = INTERFACE::getInstance(ball_radius);

void setup(void) {
    Serial.begin(1000000);
    Serial1.begin(baud_rate_simulink);

    delay(500);

    // TEST

    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);

    // TEST END

    interface.begin();
}

void loop(void) {
}