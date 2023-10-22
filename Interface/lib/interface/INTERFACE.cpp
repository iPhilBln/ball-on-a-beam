#include "INTERFACE.h"

INTERFACE& INTERFACE::getInstance(uint8_t ball_radius, float sample_time) {
    static INTERFACE _instance;

    if (_instanceInterfaceCreated == false) {
        _instance.tof        = &TOF::getInstance(ball_radius);
        _instance.ultrasonic = &HC_SR04::getInstance(ball_radius);
        _instance.stepper    = &STEPPER_ENGINE::getInstance(sample_time);

        _instance.setSampleTime(sample_time);
        _instance.setSensor(SENSOR::UNDEFINED);
        _instanceInterfaceCreated = true;
    }

    return _instance;
}

void INTERFACE::run(void) {
    while (Serial1.available()) Serial1.read(); // clear Inputbuffer
    startTimer();                               // start timer -> sample time
    _SLAVE_ENABLE();                            // enable slave and simulink communication
    SENSOR sensor_helper = getSensor();         // save selected sensor

    FLOATUNION_t val_transmit;
    FLOATUNION_t val_receive;

    switch(getSensor()) {
        case SENSOR::ULTRASONIC :   val_transmit.number[2] = ultrasonic->getDistance() / 1000.0F;  break;
        case SENSOR::TOF        :   val_transmit.number[2] = tof->getDistance() / 1000.0F;         break;
        case SENSOR::RESISTOR   :   val_transmit.number[2] = 0.0; break;
        default                 :   val_transmit.number[2] = 0.0;
    }

    delay(1000);

    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(7, INPUT);

    //Serial.println("Interface running!");

    bool init_serial1 = true;

    while (sensor_helper == getSensor()) { 
        if (communicationSimulinkSync(val_transmit, init_serial1)) {
            do {
                TCNT2 = 0;   

                digitalWrite(9, !digitalRead(9));
                digitalWrite(10, HIGH);

                setSensor(SENSOR::AUTO);

                val_receive = communicationSimulinkNew(val_transmit);

                if (static_cast<uint16_t>(val_receive.number[0]) == 0xFFFE) {
                    break; // required for synchronization of serial communication
                }
                
                stepper->setKP(val_receive.number[0]);
                stepper->setKD(val_receive.number[1]);
                stepper->setAlpha(val_receive.number[2]);

                val_transmit.number[0] = stepper->getOmega();
                val_transmit.number[1] = stepper->getAlpha();
                
                switch(getSensor()) {
                    case SENSOR::ULTRASONIC :   val_transmit.number[2] = ultrasonic->getDistanceSimulink();  break;
                    case SENSOR::TOF        :   val_transmit.number[2] = tof->getDistanceSimulink();         break;
                    case SENSOR::RESISTOR   :   val_transmit.number[2] = 0.0; break;
                    default                 :   val_transmit.number[2] = 0.0;
                }

                if (static_cast<uint16_t>(val_transmit.number[2]) == 0xFFFF) {
                    Serial.println("No sensor detected!");
                    break;
                }

                //val_receive = communicationSimulink(val_transmit);
                
                //stepper->setKP(val_receive.number[0]);
                //stepper->setKD(val_receive.number[1]);
                //stepper->setAlpha(val_receive.number[2]);
            
                // only for testing
                //stepper->setOmega(val_receive.number[2]);     // -> for testing PT1 

                while(TCNT2 < _max_TCNT) {}
            } while (sensor_helper == getSensor() && );

            init_serial1 = true;
        }
        else {
            setSensor(SENSOR::AUTO);
            init_serial1 = false;
        }

        if (static_cast<uint16_t>(val_transmit.number[2]) == 0xFFFF) {
            break;
        }
    }

    Serial.println("\nInterface has been stopped!");
}

void INTERFACE::selectSensor(void) {
    setSensor(SENSOR::UNDEFINED);

    while (getSensor() == SENSOR::UNDEFINED) {
        if (_SELECT_SENSOR_ULTRASONIC()) {
            //if( push start button) 
                setSensor(SENSOR::ULTRASONIC);
        }            
        else if (_SELECT_SENSOR_TOF()) {
            setSensor(SENSOR::TOF);
        }      
        else if (_SELECT_SENSOR_RESISTOR()) {
            setSensor(SENSOR::RESISTOR);
        }
        else {
            setSensor(SENSOR::UNDEFINED);
        }

        delay(50);
    }

    delay(1500);
}

bool INTERFACE::initSensor(void) {
    bool init_completed;

    stepper->setDegreeTarget(-20.0F);

    switch(getSensor()) {
        case SENSOR::ULTRASONIC :   init_completed = ultrasonic->beginUltrasonic();  break;
        case SENSOR::TOF        :   init_completed = tof->beginTof();                break;
        case SENSOR::RESISTOR   :   init_completed = false; break;
        default :                   init_completed = false;
    }

    stepper->setDegreeTarget(0.0F);
    while(stepper->getState()) {} // wait until engine is running   
    return init_completed;
}

void INTERFACE::setSampleTime(float sample_time) {
        uint16_t arr_prescaler[] = {1, 8, 32, 64, 128, 256, 1024};
        uint8_t  length          = sizeof(arr_prescaler) / sizeof(arr_prescaler[0]);

        for (uint8_t i = 0; i < length; i++) {
            uint32_t count_helper = static_cast<uint32_t>(sample_time * F_CPU / 1000.0F / arr_prescaler[i] - 1.0F);
            if (count_helper < 0xFF) {
                _prescaler = arr_prescaler[i];
                _max_TCNT = static_cast<uint8_t>(count_helper);
                break;
            }
        }
}

void INTERFACE::startTimer(void) {
    TCCR2A = 0;
    TCCR2B = 0;

    switch (_prescaler) {
        case 1    : TCCR2B |= _BV(CS20);              break;  // SET CS22 & CS21 & CS20 bit for prescaler 1
        case 8    : TCCR2B |= _BV(CS21);              break;  // SET CS22 & CS21 & CS20 bit for prescaler 8
        case 32   : TCCR2B |= _BV(CS21) | _BV(CS20);  break;  // SET CS22 & CS21 & CS20 bit for prescaler 32
        case 64   : TCCR2B |= _BV(CS22);              break;  // SET CS22 & CS21 & CS20 bit for prescaler 64
        case 128  : TCCR2B |= _BV(CS22) | _BV(CS20);  break;  // SET CS22 & CS21 & CS20 bit for prescaler 128
        case 256  : TCCR2B |= _BV(CS22) | _BV(CS21);  break;  // SET CS22 & CS21 & CS20 bit for prescaler 256
        case 1024 : TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20); break;  // SET CS22 & CS21 & CS20 bit for prescaler 1024
        default   : TCCR2B |= _BV(CS20);              break;  // SET CS22 & CS21 & CS20 bit for prescaler 1
    }


    //TCCR2A |= _BV(WGM21);   // Enable Clear Timer on Compare immediately 
    //TIMSK2 |= _BV(OCIE2A);  // Enable Timer 2 Compare Interrupt for OCR2A

    // 2ms cycle time -> Prescaler 128
    //OCR2A   = 249;
    //TCCR2B |= _BV(CS22) | _BV(CS20);                // Set prescaler to 128
    
    // 10ms cycle time -> Prescaler 1024   
    //OCR2A = 155;
    //TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20);  // Set prescaler to 1024
}

void INTERFACE::stopTimer(void) {
    TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20)); // Clear all CS2 bits
}

void INTERFACE::isrCycletime(void) {
    if (_instanceInterfaceCreated) {
        INTERFACE& _instance = INTERFACE::getInstance(0);
        _instance._running = false;
    }
} 

void INTERFACE::setSensor(SENSOR sensor) {
    if (sensor == SENSOR::AUTO) {
        if (_SELECT_SENSOR_ULTRASONIC())    _sensor = SENSOR::ULTRASONIC;
        else if (_SELECT_SENSOR_TOF())      _sensor = SENSOR::TOF;
        else if (_SELECT_SENSOR_RESISTOR()) _sensor = SENSOR::RESISTOR;
        else                                _sensor = SENSOR::UNDEFINED;
    }
    else {
        _sensor = sensor;
    }
}

SENSOR INTERFACE::getSensor(void) const {
    return _sensor;
}

float INTERFACE::getSampleTime(void) const {
    return _sample_time;
}

void INTERFACE::begin(void) {
    _SET_GPIO_DIRECTION_SLAVE();
    _SET_GPIO_DIRECTION_SENSOR_SELECT();

    //Serial1.setTimeout(4);
    _SERIAL0_SETUP();
    _SERIAL1_SETUP();

    while (true) {
        selectSensor();

        if (stepper->begin() == false) {
            Serial.println("Initialization for stepper engine failed!");
            continue;
        }

        if (initSensor()) {
            run();
            stop();
        }
    }
}

void INTERFACE::stop(void) {
    _SLAVE_DISABLE();
    stopTimer();
    stepper->stop();
}

