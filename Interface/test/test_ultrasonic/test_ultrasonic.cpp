#include "common.h"
#include "HC_SR04.h"

#define button 50

HC_SR04 us = {25.0};

uint16_t measurement_dist;
uint16_t distance_nr[20][6];

void setup(void) {
    Serial.begin(115200);
    delay(500);

    pinMode(button, INPUT);

    us.beginSensorLeft();
    us.beginSensorRight();
}

void loop(void) {
    uint16_t distance_l  = 0;
    uint16_t distance_r  = 0;
    uint8_t  ball_radius = us.getBallRadius();

    for (measurement_dist = ball_radius; measurement_dist <= 450 - ball_radius; measurement_dist += 10) {
        while (digitalRead(button) == HIGH) {}
        delay(500);
        
        Serial.print("# Distance = "); Serial.println(measurement_dist);

        for (uint8_t i = 0; i < 20; i++) {
            unsigned long cycle_time = millis() + 10;

            distance_l = us.getDistanceSensorLeft();
            distance_r = us.getDistanceSensorRight();

            distance_nr[i][0] = i;
            distance_nr[i][1] = measurement_dist;
            distance_nr[i][2] = distance_l;
            distance_nr[i][3] = distance_r;
            distance_nr[i][4] = distance_l < 225 ? distance_l : distance_r; // measure only the first 22.5 cm
            distance_nr[i][5] = distance_l > 225 ? distance_l : distance_r; // measure only the last  22.5 cm

            while (millis() < cycle_time) {}
        }
        
        Serial.println("# Durchgang\tdistance_soll\tdistance_left\tdistance_right\tfirst 25.5cm\tlast 25.5cm");

        for (uint8_t i = 0; i < 20; i++) {
            Serial.print(distance_nr[i][0]); Serial.print("\t"); 
            Serial.print(distance_nr[i][1]); Serial.print("\t"); 
            Serial.print(distance_nr[i][2]); Serial.print("\t"); 
            Serial.print(distance_nr[i][3]); Serial.print("\t"); 
            Serial.print(distance_nr[i][4]); Serial.print("\t"); 
            Serial.print(distance_nr[i][5]); Serial.print("\t"); 
            Serial.println();
            Serial.println();
        }
    }
}