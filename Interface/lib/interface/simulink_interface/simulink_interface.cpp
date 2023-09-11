#include "simulink_interface.h"
#include "STEPPER_ENGINE.h"

FLOATUNION_t communicationSimulink(FLOATUNION_t val_transmit) {
    static FLOATUNION_t val_receive;

    size_t input_buffer1_length = Serial1.available();

    if (input_buffer1_length >= sizeof(val_receive.buffer) + 2) {
        byte data = Serial1.read();
        unsigned long timeout = millis() + 2U;

        while (data != header) {
            data = Serial1.read();
            if (millis() > timeout) {
                Serial.println("Timeout!");
                return val_receive;
            }
        }        
        
        FLOATUNION_t val_temp;

        for (byte i = 0; i < sizeof(val_temp.buffer); i++) {
            val_temp.buffer[i] = Serial1.read();
        }

        data = Serial1.read();
        if (data != '\n') {
            Serial.println("Daten fehlerhaft.");
            return val_receive;
        }

        Serial1.write(val_transmit.buffer, sizeof(val_transmit.buffer));

        val_receive = val_temp;
    }
    return val_receive;
}