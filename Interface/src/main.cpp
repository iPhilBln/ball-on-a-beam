#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h"  //only needed with flash breakpoints


//#include "STEPPER_ENGINE.h"
//#include "HC_SR04.h"

//HC_SR04& us = HC_SR04::getInstance(20, 24.4);
//STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance(ENGINE_STEP_MODE::halfstep);

#define button 50

#include "PID.h"

CONTROLLER& pid = CONTROLLER::getInstance(SENSOR::ULTRASONIC, 1.0 , 0.0, 0.0);

void setup(void) {
     //initialize GDB stub --> Debugger
    //debug_init();

    Serial.begin(baud_rate_simulink);
    Serial1.begin(baud_rate_simulink);

    delay(500);
    pinMode(button, INPUT);
    
    pid.begin();
    //while (digitalRead(button) == HIGH) {}
    pid.testEngine();
    //HC_SR04&        ultrasonic = HC_SR04::getInstance();
    //ultrasonic.beginUltrasonic();
    //pid.runStepresponseOpenLoop();
    //pid.runStepresponseClosedLoop(225);
}

STEPPER_ENGINE& stepperN = STEPPER_ENGINE::getInstance();
void loop(void) {
    int i = -500;
    while (digitalRead(button) == HIGH) {}
    Serial.println("#time degree_target rpm_actual");

    while (i < 851) {
        switch (i) {
            case 0: stepperN.setDegreeTarget(10.0);     break;
            case 50: stepperN.setDegreeTarget(20.0);    break;
            case 250: stepperN.setDegreeTarget(45.0);    break;
            case 400: stepperN.setDegreeTarget(-30.0);  break;
            case 700: stepperN.setDegreeTarget(-45.0);  break;
        }
        Serial.println(String(i) + ";" + String(stepperN.getDegreeActual()) + ";" + String(stepperN.getRpmActual()) + ";" + String(stepperN.getDegreeTarget()));
        delay(1);
        i++;
    }
    stepperN.setDegreeTarget(0.0);


    while (digitalRead(button) == HIGH) {}
    pid.runStepresponseOpenLoop(10.0);
    while (digitalRead(button) == HIGH) {}
    pid.runStepresponseOpenLoop(20.0);
    while (digitalRead(button) == HIGH) {}
    pid.runStepresponseOpenLoop(45.0);
    while (digitalRead(button) == HIGH) {}
    pid.runStepresponseOpenLoop(-10.0);
    while (digitalRead(button) == HIGH) {}
    pid.runStepresponseOpenLoop(-20.0);
    while (digitalRead(button) == HIGH) {}
    pid.runStepresponseOpenLoop(-45.0);
    while (digitalRead(button) == HIGH) {}
}