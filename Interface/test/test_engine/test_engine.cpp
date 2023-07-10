
#include "common.h"
#include "avr8-stub.h"
#include "app_api.h"  //only needed with flash breakpoints

#define button 50

void setup() {
  //initialize GDB stub
  //debug_init();

  Serial.begin(115200);
  delay(500);
  pinMode(button, INPUT);

  Serial.println("Start");

  STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance(ENGINE_STEP_MODE::halfstep);
  stepper.begin();

  while (true) {
    while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("-45:");  //Serial.flush();
    delay(500);
    stepper.setDegreeTarget(-45.0);

    while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("stop1:");  //Serial.flush();
    delay(500);
    stepper.stop();

    while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("32:");  //Serial.flush();
    delay(500);
    stepper.setDegreeTarget(32.0);

    while (digitalRead(button) == HIGH) {};   
    //Serial.println(); //Serial.println("Stop2:");  //Serial.flush();
    delay(500); 
    stepper.stop();

    while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("32:");  //Serial.flush();
    delay(500);
    stepper.setDegreeTarget(-32.0);

    while (digitalRead(button) == HIGH) {};   
    //Serial.println(); //Serial.println("Stop2:");  //Serial.flush();
    delay(500); 
    stepper.stop(); 

    while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("32:");  //Serial.flush();
    delay(500);
    stepper.setDegreeTarget(-15.0);

      while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("32:");  //Serial.flush();
    delay(500);
    stepper.setDegreeTarget(-472.0);

    while (digitalRead(button) == HIGH) {};
    //Serial.println(); //Serial.println("32:");  //Serial.flush();
    delay(500);
    stepper.setDegreeTarget(132.0);
  } 
}

void loop() {
  
}