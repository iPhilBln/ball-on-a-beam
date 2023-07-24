#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h"  //only needed with flash breakpoints

#include "STEPPER_ENGINE.h"
#include "HC_SR04.h"

#define button 50
#define _ULTRASONIC_CHECK_LEVEL_ECHO_LEFT()  (PINL & _BV(PL0) ) // check if Pin L0 is HIGH and return TRUE
#define _ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT() (PINL & _BV(PL1) ) // check if Pin L1 is HIGH and return TRUE

HC_SR04 us = {24.4};
STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance(ENGINE_STEP_MODE::halfstep);

uint16_t measurement_dist;
uint16_t distance_nr[20][10];

void setup(void) {
     //initialize GDB stub --> Debugger
    //debug_init();

    Serial.begin(115200);
    delay(500);

    pinMode(button, INPUT);

    stepper.begin();
    stepper.setDegreeTarget(20.0);
    us.beginSensorLeft();
    stepper.setDegreeTarget(-20.0);
    us.beginSensorRight();
    stepper.setDegreeTarget(0.0);
}

void loop(void) {
    uint8_t  ball_radius = us.getBallRadius();
    
    uint16_t distance_l  = 0;
    uint16_t distance_r  = 0;
    uint16_t distance_linear_reg_l = 0;
    uint16_t distance_linear_reg_lr = 0;
    
    const float slope_linear_reg_l = 0.0;
    const float offset_linear_reg_l = 0.0;
    //const float slope_linear_reg_l = 0.9664;      -> Prototyp 2
    //const float offset_linear_reg_l = 26.0029;    -> Prototyp 2

    const float slope_linear_reg_lr = 0.0;
    const float offset_linear_reg_lr = 0.0;
    //const float slope_linear_reg_lr = 0.9420;     -> Prototyp 2
    //const float offset_linear_reg_lr = 20.7821;   -> Prototyp 2
    
    while (digitalRead(button) == HIGH) {}
    
    // Output as csv format
    Serial.print("\"measurement\";");
    Serial.print("\"distance_soll\";");
    Serial.print("\"distance_left\";");
    Serial.print("\"distance_right\";");
    Serial.print("\"first half per sensor\";");
    Serial.print("\"last half per sensor\";");
    Serial.print("\"linear regression only left\";");
    Serial.print("\"lear regression both\"");
    Serial.print("\"average\"");
    Serial.print("\"minimum\"");
    Serial.print("\"maximum\"");
    Serial.println();


    for (measurement_dist = ball_radius; measurement_dist <= static_cast<uint16_t>(450 - ball_radius); measurement_dist += 10) {
        while (digitalRead(button) == HIGH) {}
        delay(500);

        for (uint8_t i = 0; i < 20; i++) {
            while (_ULTRASONIC_CHECK_LEVEL_ECHO_LEFT() || _ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT()) {}

            unsigned long cycle_time = micros() + 11 * 1000;
            
            distance_l = us.getDistanceSensorLeft();
            while (micros() < (cycle_time - 5500)) {};
            distance_r = us.getDistanceSensorRight();
            distance_linear_reg_l = slope_linear_reg_l * distance_l + offset_linear_reg_l;
            distance_linear_reg_lr = slope_linear_reg_lr * (distance_l < 225 ? distance_l : distance_r) + offset_linear_reg_lr;


            distance_nr[i][0] = i;
            distance_nr[i][1] = measurement_dist;
            distance_nr[i][2] = distance_l;
            distance_nr[i][3] = distance_r;
            distance_nr[i][4] = distance_l < 225 ? distance_l : distance_r; // measure only the first 22.5 cm
            distance_nr[i][5] = distance_l > 225 ? distance_l : distance_r; // measure only the last 22.5 cm
            distance_nr[i][6] = distance_linear_reg_l;  // linear regression only sensor left
            distance_nr[i][7] = distance_linear_reg_lr; // linear regression both sensors and half by half

            // best corridor

            if (distance_l < 71) {
                distance_nr[i][8] = distance_l;
            }
            else if (distance_r > 371) {
                distance_nr[i][8] = distance_r;
            }
            else if (270 < distance_l && distance_l < 371) {
                distance_nr[i][8] = distance_l;
            }
            else if (70 < distance_r && distance_r < 171) {
                distance_nr[i][8] = distance_r;
            }
            else if (170 < distance_l && distance_l < 226) {
                distance_nr[i][8] = distance_l;
            }
            else if (225 < distance_r && distance_r < 271) {
                distance_nr[i][8] = distance_r;
            }
            else {
                distance_nr[i][8] = (distance_l + distance_r) >> 1;
            }

            double distance_reg = 1.011791994 * distance_nr[i][8] + 4.512277503;
            distance_nr[i][9] = static_cast<uint16_t>(distance_reg);     

            while (micros() < cycle_time) {}
        }     
        
        for (uint8_t i = 0; i < 20; i++) {
            Serial.print(distance_nr[i][0]); Serial.print(";"); 
            Serial.print(distance_nr[i][1]); Serial.print(";"); 
            Serial.print(distance_nr[i][2]); Serial.print(";"); 
            Serial.print(distance_nr[i][3]); Serial.print(";"); 
            Serial.print(distance_nr[i][4]); Serial.print(";"); 
            Serial.print(distance_nr[i][5]); Serial.print(";"); 
            Serial.print(distance_nr[i][6]); Serial.print(";");
            Serial.print(distance_nr[i][7]); Serial.print(";");
            Serial.print(distance_nr[i][8]); Serial.print(";");

            float average;
            uint16_t min;  
            uint16_t max;

            for (uint8_t j = 2; j < 10; j++) {
                average = 0.0;
                min     = 0xFFFF;  
                max     = 0;

                for (uint8_t i = 0; i < 20; i++) {
                    average += distance_nr[i][j] / 20.0;
                    min = distance_nr[i][j] < min ? distance_nr[i][j] : min;
                    max = distance_nr[i][j] > max ? distance_nr[i][j] : max;
                }
                Serial.print(average); Serial.print(";");
                Serial.print(min);     Serial.print(";");
                Serial.print(max);     Serial.print(";");
            }


            Serial.println();
        }

    }
}