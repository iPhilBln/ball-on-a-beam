#include "simulink_interface.h"

FLOATUNION_t communicationSimulink(FLOATUNION_t val_transmit) {
    static FLOATUNION_t val_receive;

    Serial1.write(245U);
    Serial1.write(val_transmit.buffer, sizeof(val_transmit.buffer));
    Serial1.write('\n');

    size_t input_buffer1_length = Serial1.available();

    if (input_buffer1_length >= sizeof(val_receive.buffer) + 2) {
        unsigned long timestamp = millis();
        unsigned long timeout   = 5U;

        byte data = Serial1.read();

        while (data != header) {
            data = Serial1.read();
            if (millis() - timestamp > timeout) {
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

        //Serial1.write(245U);
        //Serial1.write(val_transmit.buffer, sizeof(val_transmit.buffer));
        //Serial1.write('\n');

        val_receive = val_temp;
        digitalWrite(10, LOW);
    }

    return val_receive;
}

FLOATUNION_t communicationSimulinkNew(FLOATUNION_t val_transmit) {
    static uint8_t      timeout_counter = 0;
    static FLOATUNION_t val_receive;

    if (digitalRead(7) == LOW){
        timeout_counter = 0;
    }
    
    if (digitalRead(7) == HIGH && timeout_counter < 2) {
        Serial1.write(245U);
        Serial1.write(val_transmit.buffer, sizeof(val_transmit.buffer));
        Serial1.write('\n');
    }
    else {
        FLOATUNION_t val_timeout;
        val_timeout.number[0] = 0xFFFE * 1.0F;

        return val_timeout;
    }


    unsigned long timestamp = millis();
    unsigned long timeout   = 4U;

    size_t inputbuffer = Serial1.available();

    while (inputbuffer < sizeof(val_receive.buffer) + 2) {
        inputbuffer = Serial1.available();

        if (inputbuffer > 0) Serial.print(String(inputbuffer) + ' ');

        if (millis() - timestamp > timeout) {
            Serial.println("Timeout!");

            FLOATUNION_t val_timeout;
            switch (timeout_counter) {
                case 0 : timeout_counter++; break;
                case 1 : timeout_counter++; break;
                case 2 : timeout_counter++;   _SLAVE_DISABLE();  break;
                case 3 : timeout_counter = 0; _SLAVE_ENABLE();   break;
                default:                    break;
            }

            timeout_counter == 0 ? val_timeout.number[0] = 0xFFFE * 1.0F : '';
            return val_timeout;
        }
    }

    Serial.println();
    digitalWrite(10, LOW);

    byte data = Serial1.read();
    while (data != header) {
        data        = Serial1.read();
        inputbuffer = Serial1.available();

        if (inputbuffer == 0) {
            Serial.println("No valid data!"); 
            return val_receive;
        }
    }        

    inputbuffer = Serial1.available();
    if (inputbuffer < sizeof(val_receive.buffer)) return val_receive;
    
    FLOATUNION_t val_temp;
    for (byte i = 0; i < sizeof(val_temp.buffer); i++) {
        val_temp.buffer[i] = Serial1.read();
    }

    data = Serial1.read();
    if (data != '\n') {
        Serial.println("Daten fehlerhaft.");
        return val_receive;
    }

    val_receive     = val_temp;
    timeout_counter = 0;

    return val_receive;
}

bool communicationSimulinkSync(FLOATUNION_t val_transmit, bool init_serial1) {
    unsigned long timestamp_sync = millis();
    unsigned long timeout_sync   = 500U;
    
    if (init_serial1) {
        Serial1.write(245U);
        Serial1.write(val_transmit.buffer, sizeof(val_transmit.buffer));
        Serial1.write('\n');

        Serial.print("Waiting for connection .");
    }

    while (digitalRead(7) == LOW) {
        
        if (millis() - timestamp_sync > timeout_sync) {
            Serial.print(".");
            return false;
        }
    }
    Serial.println("\nConnection established!");

    INTERFACE& interface = INTERFACE::getInstance();

    uint16_t delay_time = static_cast<uint16_t>(interface.getSampleTime() * 1000.0F * 1000.0F) - 1500U;

    delayMicroseconds(delay_time);
    return true;
}