
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

}

void loop() {
  
}