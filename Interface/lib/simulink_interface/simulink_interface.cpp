#include "simulink_interface.h"

float recSimu(void) {
    FLOATUNION_t val;
    if (Serial1.available() >= sizeof(float) + 2) {
        byte data = Serial1.read();
        unsigned long timeout = millis() + 1U;

        while (data != header) {
            data = Serial1.read();
            if (millis() > timeout) {
                Serial.println("Timeout!");
                return NAN;
            }
        }

        Serial1.setTimeout(1);
        if (Serial1.readBytesUntil('\n', val.buffer, sizeof(float)) < sizeof(float)) {
            Serial.println("Daten fehlerhaft.");
            return NAN;
        }
        
        /*
        for (byte i = 0; i < sizeof(val.buffer); i++) {
            val.buffer[i] = Serial1.read();
        }

        data = Serial1.read();
        if (data != '\n') {
            Serial.println("Daten fehlerhaft.");
            return NAN;
        }
        */
        Serial.println("Empangen: " + String(val.number[0], 3) + " Buffersize: " + String(Serial1.available()));
        return val.number[0];
    }

    return NAN;
}

void sendSimu(FLOATUNION_t val) {
    Serial1.write(header);
    Serial1.write(val.buffer, sizeof(val.buffer));
    Serial1.write('\n');
}

/*

void transmitFloatToSimulink(float transVal) {
    FLOATUNION_t val;
    val.number = transVal;
    
    Serial1.write(header);                      // Startbedingung -> wichtig für Synchronisierung
    Serial1.write(val.buffer, sizeof(val.number));   // gesendeter Wert als Float
    Serial1.write('\n');                            // Terminator -> wichtig für Synchronisierung
}

float receiveFloatFromSimulink(void) {
    static FLOATUNION_t val;

    // Startbedingung -> wichtig für Synchronisierung
    if (Serial1.available() < 1) return val.number;


    // warte bis kompletter wert übertragen wurde oder werfe timeout nach 0,5ms
    unsigned long timeout = micros() + 500U;
    bool data_exists = false;

    while (Serial1.available() < sizeof(val.number) + 1) {
        while (data_exists == false) {
            // Suche Startbyte im Inputbuffer
            byte data = Serial1.read();
            if (data == header) data_exists = true;
            if (micros() > timeout) return val.number;
        }

        if (micros() > timeout) return val.number;
    }

    FLOATUNION_t val;
    Serial1.readBytes(val.bytes, sizeof(val));
    
    byte data_termination = Serial1.read();
    
    if(data_termination == '\n'){
        val = val;
        Serial.println("Empfangen: " + String(val.number,3));
    }
    return val.number;
}
*/
/*
float receiveFloatFromSimulink(void) {
    static FLOATUNION_t val;

    // Startbedingung -> wichtig für Synchronisierung
    // suche im Serial 1 Inputbuffer nach Startzeichen
    bool dataReceived = false;
    while (Serial1.available() > 0) {
        byte data = Serial1.read();
        if (data  == header) {
            dataReceived = true;
            break;
        }
        //Serial.println("F");
    }

    // falls kein Wert gefunden wurde gebe alten Wert zurück
    if (dataReceived == false) {
        //Serial.println("Keine Daten erhalten!");
        return val.number;
    }

    // warte bis kompletter Wert übertragen wurde oder werfe Timeout nach 0,5ms
    unsigned long timeout = micros() + 500U;
    while (Serial1.available() < (sizeof(float) + 1)) {
        if (micros() > timeout) {
            Serial.println("Timeout!");
            return val.number;
        }
    }

    val.length = sizeof(val.number) / sizeof(val.buffer[0]);
    
    for (byte i = 0; i < val.length; i++) {
        val.buffer[i] = Serial1.read();
        Serial.println(val.buffer[i]);
    }      
    
    char dataChar = Serial1.read();
    if (dataChar == '\n')
        Serial.println("Empfangen: " + String(val.number,3));

    return val.number;
}

*/