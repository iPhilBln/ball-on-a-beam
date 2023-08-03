#include "PID.h"

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

CONTROLLER& CONTROLLER::getInstance(SENSOR sensor, float kp, float ki, float kd, bool force) {
    static CONTROLLER _instance;

    if (_instanceCreated == false || force) {
        _instance.setSensor(sensor);
        _instance.setKp(kp);
        _instance.setKi(ki);
        _instance.setKd(kd);

        _instanceCreated = true;
    }

    return _instance;
}

void CONTROLLER::startTimer(void) {
    TCCR2A = 0;
    TCCR2B = 0;

    TCCR2A |= _BV(WGM21);   // Enable Clear Timer on Compare immediately 
    TIMSK2 |= _BV(OCIE2A);  // Enable Timer 2 Compare Interrupt for OCR2A

    // 2ms cycle time -> Prescaler 128
    OCR2A   = 249;
    TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20)); // Clear all CS2 bits
    TCCR2B |= _BV(CS22) | _BV(CS20);                // Set prescaler to 128
}

void CONTROLLER::stopTimer(void) {
    TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20)); // Clear all CS2 bits
}

void CONTROLLER::isrCycletime(void) {
    if (_instanceCreated) {
        CONTROLLER& _instance = CONTROLLER::getInstance(SENSOR::undefined,0,0,0);
        _instance._running = false;
    }
} 

void CONTROLLER::setSensor(SENSOR sensor) {
    _sensor = sensor;
}

void CONTROLLER::setKp(float kp) {
    _kp = kp;
}

void CONTROLLER::setKi(float ki) {
    _ki = ki;
}

void CONTROLLER::setKd(float kd) {
    _kd = kd;
}

SENSOR CONTROLLER::getSensor(void) {
    return _sensor;
}

float CONTROLLER::getKp(void) {
    return _kp;
}

float CONTROLLER::getKi(void) {
    return _ki;
}

float CONTROLLER::getKd(void) {
    return _kd;
}

void CONTROLLER::begin(void) {
    stepper.begin();
    startTimer();

    Serial1.setTimeout(2);
    _SERIAL0_SETUP();
    _SERIAL1_SETUP();

    DDRC |= _BV(PC5);
    PORTC |= _BV(PC5);
}

void CONTROLLER::stop(void) {
    stepper.stop();
    stopTimer();
    PORTC &= ~( _BV(PC5) );
}

void CONTROLLER::runPcontroller(void) {
    switch (_sensor) {
        case SENSOR::CAPAITY :    break;
        case SENSOR::RESISTOR :   break;
        case SENSOR::TOF :        break;
        case SENSOR::ULTRASONIC : break;//ultrasonicPtr = &HC_SR04::getInstance(); break;
        case SENSOR::undefined :  break;
    }

    while (true) {

    }
}

void CONTROLLER::runStepresponseClosedLoop(uint16_t setpoint) {
    switch (_sensor) {
        case SENSOR::CAPAITY :    break;
        case SENSOR::RESISTOR :   break;
        case SENSOR::TOF :        break;
        case SENSOR::ULTRASONIC : break;//ultrasonicPtr = &HC_SR04::getInstance(); break;
        case SENSOR::undefined :  break;
    }
    Serial.println("Start");
    HC_SR04&        ultrasonic = HC_SR04::getInstance();
    ultrasonic.beginUltrasonic();

    while (true) {
        _running = true;

        double time = 0;
        double kp = static_cast<double>(1.0 * analogRead(A0));
        double ki = 0.0;
        double kd = 0.0;
        //double ki = static_cast<double>(1.0 * analogRead(A1));
        //double kd = static_cast<double>(1.0 * analogRead(A2));

        kp = mapf(kp, 0, 1023, 0.0, 100.0);
        ki = mapf(ki, 0, 1023, 0.0, 100.0);
        kd = mapf(kd, 0, 1023, 0.0, 100.0);

        Serial.println(String(kp,3) + "\t" + String(ki,3) + "\t" +String(kd,3));

        uint16_t distance;
        if (_sensor == SENSOR::CAPAITY) {

        }
        else if (_sensor == SENSOR::RESISTOR) {

        }
        else if (_sensor == SENSOR::TOF) {

        }
        else if (_sensor == SENSOR::ULTRASONIC) {
            distance = ultrasonic.getDistance();
            uint16_t prescaler = 1024;
            time = static_cast<double>(((1.0 * OCR2A * prescaler) + 1.0) / (1.0 * F_CPU));
        }

        _error = static_cast<double>(1.0 * setpoint - 1.0 * distance);

        double pPart = kp * _error;

        _integral += _error * time;
        double iPart = ki * _integral;

        _derivate = (_error - _pre_error) / time;
        double dPart = kd * _derivate;

        _pre_error = _error;

        double sum = pPart + iPart + dPart;

        if (sum > 450.0)  sum = 450.0;
        if (sum < -450.0) sum = -450.0;

        sum = mapf(sum, -450.0, 450.0, 45.0, -45.0);

        stepper.setDegreeTarget(sum);
        
        while (_running) {}
    }
}

void CONTROLLER::runStepresponseOpenLoop(float step) {
    //void * sensor = nullptr;
    uint32_t cycle_counter = 0;

    switch (_sensor) {
        case SENSOR::CAPAITY :    break;
        case SENSOR::RESISTOR :   break;
        case SENSOR::TOF :        break;
        case SENSOR::ULTRASONIC : break;// HC_SR04&  sensor = HC_SR04::getInstance();break; //ultrasonicPtr = &HC_SR04::getInstance(); break;
        case SENSOR::undefined :  break;
    }
    double time;
    uint16_t distance;

    if (_sensor == SENSOR::CAPAITY) {

    }
    else if (_sensor == SENSOR::RESISTOR) {

    }
    else if (_sensor == SENSOR::TOF) {

    }
    else if (_sensor == SENSOR::ULTRASONIC) {
        HC_SR04& ultrasonic = HC_SR04::getInstance();

        uint16_t prescaler = 1024;
        stepper.setDegreeTarget(0.0);
        while (stepper.getState()) {}

        Serial.println("#time / stepper degree / ultrasonic_distance"); Serial.flush();
        delay(2000);
        stepper.setDegreeTarget(step);

        while (cycle_counter < 400) {
            _running = true;

            distance = ultrasonic.getDistance();
            time = static_cast<double>(((1.0 * OCR2A * prescaler) + 1.0) / (1.0 * F_CPU));

            Serial.println(String(time * cycle_counter++) + ";" + String(stepper.getDegreeActual()) + ";" + String(distance));
            while (_running);
        }
        stepper.setDegreeTarget(0.0);
        while (stepper.getState()) {}
    }

}

void CONTROLLER::testEngine(void) {
    Serial.println("Start Test...");    
    static float alpha = 0.0;
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);

    while (true) {
        _running = true;
        digitalWrite(9, !digitalRead(9));

        FLOATUNION_t val_transmit;
        val_transmit.number[0] = stepper.getFreq();
        val_transmit.number[1] = stepper.getDegreeActual();
        val_transmit.number[2] = 0.0;
        val_transmit.number[3] = 0.0;
        
        float alpha_receive = communicationSimulink(val_transmit);

        isnan(alpha_receive) ? alpha = alpha : alpha = alpha_receive;

        stepper.setFreq(alpha);
        
        
        //transmitFloatToSimulink(degree);
        //transmitFloatToSimulink(rpm);

        //sendSimu(val_transmit);
        
        while(_running) {}
    }
    Serial.println("Ende");
}