#include "INTERFACE.h"

INTERFACE& INTERFACE::getInstance(uint8_t ball_radius) {
    static INTERFACE _instance;

    if (_instanceInterfaceCreated == false) {
        _instance.tof        = &TOF::getInstance(ball_radius);
        _instance.ultrasonic = &HC_SR04::getInstance(ball_radius);
        _instance.stepper    = &STEPPER_ENGINE::getInstance();

        _instance.setSensor(SENSOR::UNDEFINED);
        _instanceInterfaceCreated = true;
    }

    return _instance;
}

void INTERFACE::run(void) {
    while (Serial1.available()) Serial1.read(); // clear Inputbuffer
    startTimer();                               // start timer -> 2ms cycle time
    _SLAVE_ENABLE();                            // enable slave and simulink communication
    SENSOR sensor_helper = getSensor();         // save selected sensor

    pinMode(10, INPUT_PULLUP); 

    //while (sensor_helper == getSensor()) {    
    while (digitalRead(10) == HIGH) {
        _running = true;

        FLOATUNION_t val_transmit;
        FLOATUNION_t val_receive;

        val_transmit.number[0] = stepper->getOmega();
        val_transmit.number[1] = stepper->getAlpha();

        switch(getSensor()) {
            case SENSOR::ULTRASONIC :   val_transmit.number[2] = ultrasonic->getDistanceSimulink();  break;
            case SENSOR::TOF        :   val_transmit.number[2] = tof->getDistanceSimulink();         break;
            case SENSOR::RESISTOR   :   val_transmit.number[2] = 0.0; break;
            default                 :   val_transmit.number[2] = 0.0;
        }

        val_receive = communicationSimulink(val_transmit);

        stepper->setKP(val_receive.number[0]);
        stepper->setT1(val_receive.number[1]);
        stepper->setAlpha(val_receive.number[2]);
        
        if (_SELECT_SENSOR_ULTRASONIC())    setSensor(SENSOR::ULTRASONIC);
        else if (_SELECT_SENSOR_TOF())      setSensor(SENSOR::TOF);
        else if (_SELECT_SENSOR_RESISTOR()) setSensor(SENSOR::RESISTOR);
        else                                setSensor(SENSOR::UNDEFINED);

        while(_running) {}
    }
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

        delay(100);
    }
}

void INTERFACE::initSensor(void) {
    switch(getSensor()) {
        case SENSOR::ULTRASONIC :   ultrasonic->beginUltrasonic();  break;
        case SENSOR::TOF        :   tof->beginTof();                break;
        case SENSOR::RESISTOR   :   while (true) {}; break;
    }
}

void INTERFACE::startTimer(void) {
    TCCR2A = 0;
    TCCR2B = 0;

    TCCR2A |= _BV(WGM21);   // Enable Clear Timer on Compare immediately 
    TIMSK2 |= _BV(OCIE2A);  // Enable Timer 2 Compare Interrupt for OCR2A

    // 2ms cycle time -> Prescaler 128
    OCR2A   = 249;
    TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20)); // Clear all CS2 bits
    TCCR2B |= _BV(CS22) | _BV(CS20);                // Set prescaler to 128
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
    _sensor = sensor;
}

SENSOR INTERFACE::getSensor(void) const {
    return _sensor;
}

void INTERFACE::begin(void) {
    _SET_GPIO_DIRECTION_SLAVE();
    _SET_GPIO_DIRECTION_SENSOR_SELECT();

    Serial1.setTimeout(2);
    _SERIAL0_SETUP();
    _SERIAL1_SETUP();

    stepper->begin();

    while (true) {
        selectSensor();
        initSensor();
        run();
        stop();
    }
}

void INTERFACE::stop(void) {
    _SLAVE_DISABLE();
    stepper->stop();
    stopTimer();
}

