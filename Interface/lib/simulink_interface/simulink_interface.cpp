#include "simulink_interface.h"
#include "STEPPER_ENGINE.h"

float recSimu(void) {
    FLOATUNION_t val_receive;

    if (Serial1.available() >= sizeof(float) + 2) {
        byte data = Serial1.read();
        unsigned long timeout = millis() + 2U;

        while (data != header) {
            data = Serial1.read();
            if (millis() > timeout) {
                Serial.println("Timeout!");
                return NAN;
            }
        }      
        
        for (byte i = 0; i < sizeof(val_receive.buffer); i++) {
            val_receive.buffer[i] = Serial1.read();
        }

        data = Serial1.read();
        if (data != '\n') {
            Serial.println("Daten fehlerhaft.");
            return NAN;
        }

        Serial.println("Empangen: " + String(val_receive.number[0], 3) + " Buffersize: " + String(Serial1.available()));
        return val_receive.number[0];
    }

    return NAN;
}

void sendSimu(FLOATUNION_t val_transmit) {
    //Serial1.write(header);
    Serial1.write(val_transmit.buffer, sizeof(val_transmit.buffer));
    //Serial.println("Gesendet: " + String(val_receive.number[0], 3) +",\t" + String(val_receive.number[1], 3));
    //Serial1.write('\n');
}

float communicationSimulink(FLOATUNION_t val_transmit) {
    FLOATUNION_t val_receive;

    if (Serial1.available() >= sizeof(val_receive.number[0]) + 2) {
        digitalWrite(8, HIGH);
        byte data = Serial1.read();
        unsigned long timeout = millis() + 2U;

        while (data != header) {
            data = Serial1.read();
            if (millis() > timeout) {
                Serial.println("Timeout!");
                return NAN;
            }
        }        
        
        for (byte i = 0; i < sizeof(val_receive.number[0]); i++) {
            val_receive.buffer[i] = Serial1.read();
        }

        data = Serial1.read();
        if (data != '\n') {
            Serial.println("Daten fehlerhaft.");
            return NAN;
        }

        
        //Serial.println("Empangen: " + String(val_receive.number[0], 3) + " Buffersize: " + String(Serial1.available()));
        //STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance();
        //if( stepper.getFreq() > 0.0 && stepper.getFreq() < 49.5) {
        //    Serial.println("Empangen: " + String(val_receive.number[0], 3) + " Buffersize: " + String(Serial1.available()));
        //    Serial.println("Gesendet: " + String(val_transmit.number[0], 3) +",\t" + String(val_transmit.number[1], 3));
        //}
        
        sendSimu(val_transmit);
        digitalWrite(8, LOW);
        return val_receive.number[0];
    }

    return NAN;
}