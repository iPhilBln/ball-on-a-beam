#include <Arduino.h>
#include "INTERFACE.h"

//#include <Wire.h>
//#include <VL53L0X.h>

//VL53L0X sensor;

const uint8_t ball_radius = 20;
INTERFACE& interface = INTERFACE::getInstance(ball_radius);

void setup(void) {
    Serial.begin(1000000);
    Serial1.begin(baud_rate_simulink);

    delay(500);


    // TEST
    /*
    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {}
    }

    sensor.setMeasurementTimingBudget(99 * 1000U); // 99 ms timing budget
    Serial.print(sensor.readRangeSingleMillimeters());
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    while (true) {}
    */
    TOF& sensor = TOF::getInstance();
    sensor.beginTof();

    // TEST END

    interface.begin();
}

void loop(void) {
}