#include <Arduino.h>
#include "INTERFACE.h"

#define button 6

const uint8_t ball_radius = 20;                 // ball radius in mm
const float   sample_time = 7.5F / 1000.0F;     // sample time in s

void setup(void) {
    Serial.begin(baud_rate_simulink);
    Serial1.begin(baud_rate_simulink);

    delay(500);

    INTERFACE& interface = INTERFACE::getInstance(ball_radius, sample_time);
    interface.begin();
}


void loop(void) {

    // TEST SENSORIK

    /*
    INTERFACE& interface = INTERFACE::getInstance(ball_radius);

    uint16_t measurement_dist;
    uint16_t distance_nr[20][3];
    
    while (interface.ultrasonic->beginUltrasonic() == false);

    // Output as csv format
    Serial.print("\"measurement\";");
    Serial.print("\"distance_soll\";");
    Serial.print("\"distance_us\";");

    Serial.println();

    for (int i = 20 ; i < 450; i += 10) {
        while (digitalRead(button) == HIGH) {}
        delay(500);

        for (uint8_t j = 0; j < 20; j++) {
            distance_nr[j][0] = j;
            distance_nr[j][1] = i;
            
            uint16_t distance = interface.ultrasonic->getDistancePure();
            uint16_t offset   = interface.ultrasonic->getOffset();

            delay(100);
            if (distance == 0) {
                distance_nr[j][2] = 500;
            }
            else {
                distance_nr[j][2] = ball_radius + (distance - offset);
            }
        }

        for (uint8_t j = 0; j < 20; j++) {
            Serial.print(distance_nr[j][0]); Serial.print(";"); 
            Serial.print(distance_nr[j][1]); Serial.print(";"); 
            Serial.print(distance_nr[j][2]); Serial.print(";"); 

            Serial.println();
        }
    }

    
    while (digitalRead(button) == HIGH) {}
    delay(500);

    while (interface.tof->beginTof() == false);

    // Output as csv format
    Serial.print("\"measurement\";");
    Serial.print("\"distance_soll\";");
    Serial.print("\"distance_tof\";");

    Serial.println();

    for (int i = 20 ; i < 450; i += 10) {
        while (digitalRead(button) == HIGH) {}
        delay(500);
        
        interface.tof->startMeasurement();

        for (uint8_t j = 0; j < 20; j++) {
            delay(60);
            uint16_t distance = interface.tof->getDistancePure();
            uint16_t offset   = interface.tof->getOffset();

            distance_nr[j][0] = j;
            distance_nr[j][1] = i;
            distance_nr[j][2] = ball_radius + (distance - offset);
        }

        for (uint8_t j = 0; j < 20; j++) {
            Serial.print(distance_nr[j][0]); Serial.print(";"); 
            Serial.print(distance_nr[j][1]); Serial.print(";"); 
            Serial.print(distance_nr[j][2]); Serial.print(";"); 

            Serial.println();
        }
    }
    */
}
