#ifndef SIMULINK_INTERFACE_H_
#define SIMULINK_INTERFACE_H_

#include <Arduino.h>

#define baud_rate_simulink 115200
#define header 0b11110101

// Create a union to easily convert float to byte
typedef union{
  float number[4];
  byte  buffer[16];
} FLOATUNION_t;

void    transmitFloatToSimulink(float val);
float   receiveFloatFromSimulink(void);

void sendSimu(FLOATUNION_t val);
float recSimu(void);

#endif