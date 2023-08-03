#ifndef SIMULINK_INTERFACE_H_
#define SIMULINK_INTERFACE_H_

#include <Arduino.h>

#define  _SERIAL0_SETUP() DDRE &= ~( _BV(PE0) ); DDRE |= _BV(PE1); PORTE |= _BV(PE1) | _BV(PE0); // enable Pull-Up -> prevent crosstalk
#define  _SERIAL1_SETUP() DDRD &= ~( _BV(PD2) ); DDRD |= _BV(PD3); PORTD |= _BV(PD3) | _BV(PD2); // enable Pull-Up -> prevent crosstalk

#define baud_rate_simulink 115200
#define header 0b11110101

// Create a union to easily convert float to byte
typedef union{
  float number[4];
  byte  buffer[16];
} FLOATUNION_t;

void    transmitFloatToSimulink(float val);
float   receiveFloatFromSimulink(void);

void  sendSimu(FLOATUNION_t val);
float recSimu(void);

float communicationSimulink(FLOATUNION_t val);

#endif