#include "HC_SR04.h"

HC_SR04& HC_SR04::getInstance(uint8_t ball_radius, double  temperature, uint8_t timer_prescaler) {
    static HC_SR04 _instance;

    if (_instanceCreated == false) {
        _instance.setBallRadius(ball_radius);
        _instance.setTemperature(temperature);
        _instance.setTimerPrescaler(timer_prescaler);

        _instanceCreated = true;
    }
    return _instance;
}

void    HC_SR04::setOffsetLeft(void) {
    uint8_t   counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    while (counter < 250U) {
        unsigned long cycle_time = micros() + 5500U; // 5,5ms per measurement cycle

        distance = (getDistanceSensorLeft() - 0U - _ball_radius);
        (0U < distance && distance < 150U) && (distance_old - 1 <= distance || distance <= distance_old + 1) ? counter++ : counter = 0;
        distance_old = distance;
        while (micros() < cycle_time) {}
    }
    _offset_left = static_cast<uint8_t>(distance);

    Serial.println("Offset left: " + String(_offset_left));
}

void    HC_SR04::setOffsetRight(void) {
    uint8_t   counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    while (counter < 250U) {
        unsigned long cycle_time = micros() + 5500U; // 5,5ms per measurement cycle

        distance = 450U - _ball_radius - getDistanceSensorRight();
        (0U < distance && distance < 150U) && (distance_old - 1 <= distance || distance <= distance_old + 1) ? counter++ : counter = 0;
        distance_old = distance;
        while (micros() < cycle_time) {}
    }
    _offset_right = static_cast<uint8_t>(distance);

    Serial.println("Offset right: " + String(_offset_right));
}

void    HC_SR04::setUltrasonicSpeed(double temperature) {
    _ultrasonic_speed = sqrt(1.402F * 8.3145F * (273.15F + temperature) / 0.02896F);
}

void HC_SR04::beginSensorLeft(void) {
    stopTimerEchoLeft();

    TCCR4A = 0;
    TCCR4B = _BV(ICNC4) | _BV(ICES4) | _BV(WGM42);
    TCCR4C = 0;
    TIMSK4 = _BV(ICIE4) | _BV(OCIE4A);

    switch (_timer_prescaler) {
        case 1 : OCR4A = static_cast<uint16_t>(3.5F * 16000 - 1);   break;
        case 8 : OCR4A = static_cast<uint16_t>(3.5F * 8000 - 1);    break;
    }

    _SET_GPIO_DIRECTION_ULTRASONIC_ECHO_LEFT();
    _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_LEFT();

    setOffsetLeft();
}

void HC_SR04::beginSensorRight(void) {
    stopTimerEchoRight();

    TCCR5A = 0;
    TCCR5B = _BV(ICNC5) | _BV(ICES5) | _BV(WGM52);
    TCCR5C = 0;
    TIMSK5 = _BV(ICIE5) | _BV(OCIE5A);

    switch (_timer_prescaler) {
        case 1 : OCR5A = static_cast<uint16_t>(3.5F * 16000 - 1);   break;
        case 8 : OCR5A = static_cast<uint16_t>(3.5F * 8000 - 1);    break;
    }

    _SET_GPIO_DIRECTION_ULTRASONIC_ECHO_RIGHT();
    _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_RIGHT();

    setOffsetRight();
}

void   HC_SR04::startTimerEchoLeft(void){
    TCNT4   = 0;            // reset counter value timer 4
    TCCR4B |= _BV(ICES4);   // capture TCNT4 on rising edge at first
    
    switch (_timer_prescaler) {
        case 1 : TCCR4B |= _BV(CS40);              break;
        case 8 : TCCR4B |= _BV(CS41) | _BV(CS40);  break;
    }

    _echo_timeout = false;
    _ULTRASONIC_TRIGGER_LEFT_START();
}

void   HC_SR04::stopTimerEchoLeft(void){
    TCCR4B &= ~( _BV(CS42) | _BV(CS41) | _BV(CS40) );
    _ULTRASONIC_TRIGGER_LEFT_STOP();
}

void   HC_SR04::startTimerEchoRight(void){
    TCNT5   = 0;            // reset counter value timer 5
    TCCR5B |= _BV(ICES5);   // capture TCNT5 on rising edge at first

    switch (_timer_prescaler) {
        case 1 : TCCR5B |= _BV(CS50);              break;
        case 8 : TCCR5B |= _BV(CS51) | _BV(CS50);  break;
    }

    _echo_timeout = false;
    _ULTRASONIC_TRIGGER_RIGHT_START();
}

void   HC_SR04::stopTimerEchoRight(void){
    TCCR5B &= ~( _BV(CS52) | _BV(CS51) | _BV(CS50) );
    _ULTRASONIC_TRIGGER_RIGHT_STOP();
}

void HC_SR04::setDistance(void) {
    uint16_t      distance_local = 0;

    if (_distance_left < 71U) {
        distance_local = _distance_left;
    }
    else if (_distance_right > 371U) {
        distance_local = _distance_right;
    }
    else if (270U < _distance_left && _distance_left < 371U) {
        distance_local = _distance_left;
    }
    else if (70U < _distance_right && _distance_right < 171U) {
        distance_local = _distance_right;
    }
    else if (170U < _distance_left && _distance_left < 226U) {
        distance_local = _distance_left;
    }
    else if (225U < _distance_right && _distance_right < 271U) {
        distance_local = _distance_right;
    }
    else {
        distance_local = (_distance_left + _distance_right) >> 1;
    }

    distance_local < 450U - _ball_radius ? _distance = 450 - distance_local : _distance;
    Serial.println(String(distance_local) + '\t' + String(_distance));
}


void HC_SR04::isrEchoLeftCapture(void) {
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        if (TCCR4B & _BV(ICES4)) _instance._echo_counter_left = TCNT4;                                      // capture rising edge
        else                     _instance._echo_counter_left = TCNT4 - _instance._echo_counter_left;       // capture falling edge

        TCCR4B &= ~( _BV(ICES4) );
    }
}

void HC_SR04::isrEchoLeftTimeout(void) {
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        _instance._echo_timeout = true;
    }
}

void HC_SR04::isrEchoRightCapture(void) {
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        if (TCCR5B & _BV(ICES5)) _instance._echo_counter_right = TCNT5;                                     // capture rising edge
        else                     _instance._echo_counter_right = TCNT5 - _instance._echo_counter_right;     // capture falling edge

        TCCR5B &= ~( _BV(ICES5) );
    }
}

void HC_SR04::isrEchoRightTimeout(void) {
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        _instance._echo_timeout = true;
    }
}

/*      SETTER      */

void    HC_SR04::setTimerPrescaler(uint8_t timer_prescaler) {
    _timer_prescaler = timer_prescaler;
}

void    HC_SR04::setBallRadius(uint8_t ball_radius) {
    _ball_radius = ball_radius;
}

void    HC_SR04::setTemperature(double temperature) {
    // hier noch Sensor implementieren 
    _temperature = temperature;
    setUltrasonicSpeed(_temperature); 
}

/*      GETTER      */

uint8_t HC_SR04::getBallRadius(void) const {
    return _ball_radius;
}

float  HC_SR04::getTemperature(void) const {
    return _temperature;
}

float  HC_SR04::getUltrasonicSpeed(void) const {
    return _ultrasonic_speed;
}

uint8_t HC_SR04::getTimerPrescaler(void) const {
    return _timer_prescaler;
}

uint8_t HC_SR04::getOffsetLeft(void) const {
    return _offset_left;
}

uint8_t HC_SR04::getOffsetRight(void) const {
    return _offset_right;
}

uint16_t HC_SR04::getDistanceSensorLeft(void) {
    if (_ULTRASONIC_CHECK_LEVEL_ECHO_LEFT()) return _distance_left;

             _echo_counter_left  = 0;
    uint16_t echo_counter_local  = 0;
    uint32_t distance_local      = 0;

    startTimerEchoLeft();

    while (_echo_counter_left == 0) {
        if (_echo_timeout) {
            stopTimerEchoLeft();
            //Serial.println("Timeout left rising edge!");
            return _distance_left;
        }
    }

    echo_counter_local = _echo_counter_left;
    _echo_counter_left = 0;

    while (_echo_counter_left == 0) {
        if (_echo_timeout) {
            stopTimerEchoLeft();
            //Serial.println("Timeout left falling edge!");
            return _distance_left;
        }
    }
    stopTimerEchoLeft();
    
    distance_local  = _echo_counter_left - echo_counter_local;
    distance_local *= _ultrasonic_speed; 
    distance_local /= (32000U / _timer_prescaler);
    distance_local  = 0U + _ball_radius + (distance_local - _offset_left);

    return (distance_local <= 450U) ? _distance_left = static_cast<uint16_t>(distance_local) : _distance_left; 
}

uint16_t HC_SR04::getDistanceSensorRight(void) {
    if (_ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT()) return _distance_right;

             _echo_counter_right = 0;
    uint16_t echo_counter_local  = 0;
    uint32_t distance_local      = 0;

    startTimerEchoRight();

    while (_echo_counter_right == 0U) {
        if (_echo_timeout) {
            stopTimerEchoLeft();
            //Serial.println("Timeout right rising edge!");
            return _distance_right;
        }
    }

    echo_counter_local = _echo_counter_right;
    _echo_counter_right      = 0;

    while (_echo_counter_right == 0U) {
        if (_echo_timeout) {
            stopTimerEchoRight();
            //Serial.println("Timeout right falling edge!");
            return _distance_right;
        }
    }

    stopTimerEchoRight();

    distance_local  = _echo_counter_right - echo_counter_local;
    distance_local *= _ultrasonic_speed; 
    distance_local /= (32000U / _timer_prescaler);
    distance_local  = 450U - _ball_radius - (distance_local - _offset_right);

    return (distance_local <= 450U) ? _distance_right = static_cast<uint16_t>(distance_local) : _distance_right; 
}

uint16_t HC_SR04::getDistance(void) {
    unsigned long delaySensor    = micros() + 5500U; 
    
    getDistanceSensorLeft();
    while (micros() < delaySensor) {};
    getDistanceSensorRight();

    setDistance();
    return _distance;
}

float HC_SR04::getDistanceSimulink(void) {
    static uint8_t counter = 0;

    if (counter == 0U) {         // 0ms
        if (_ULTRASONIC_CHECK_LEVEL_ECHO_LEFT() || _ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT()) {
            counter = 0; 

            stopTimerEchoLeft();
            stopTimerEchoRight();

            return _distance / 1000.0F;
        }

        startTimerEchoLeft();
    }
    else if (counter == 3U) {    // 6ms 
        if (_ULTRASONIC_CHECK_LEVEL_ECHO_LEFT() || _ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT()) {
            counter = 0;

            stopTimerEchoLeft();
            stopTimerEchoRight();

            return _distance / 1000.0F;
        }

        stopTimerEchoLeft();

        uint32_t distance_local;
        distance_local  = _echo_counter_left;
        distance_local *= _ultrasonic_speed; 
        distance_local /= (32000U / _timer_prescaler);
        distance_local  = 0U + _ball_radius + (distance_local - _offset_left);

        (distance_local <= 450U) ? _distance_left = static_cast<uint16_t>(distance_local) : _distance_left = _distance_left; 

        startTimerEchoRight();
    }
    else if (counter == 6U) {    // 12 ms
        if (_ULTRASONIC_CHECK_LEVEL_ECHO_LEFT() || _ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT()) {
            counter = 0;

            stopTimerEchoLeft();
            stopTimerEchoRight();

            return _distance / 1000.0F;
        }

        stopTimerEchoRight();

        uint32_t distance_local;
        distance_local  = _echo_counter_right;
        distance_local *= _ultrasonic_speed; 
        distance_local /= (32000U / _timer_prescaler);
        distance_local  = 450U - _ball_radius - (distance_local - _offset_right);

        (distance_local <= 450U) ? _distance_right = static_cast<uint16_t>(distance_local) : _distance_right; 

        setDistance();
    }

    counter < 6 ? counter++ : counter = 0;
    return _distance / 1000.0F;
}

void HC_SR04::beginUltrasonic(void) {
    stepper.setDegreeTarget(20.0F);
    Serial.println("Begin left.");
    beginSensorLeft();
    stepper.setDegreeTarget(-20.0F);
    Serial.println("Begin right.");
    beginSensorRight();
    stepper.setDegreeTarget(0.0F);
    
    _distance = 0;
    
    while(stepper.getState()) {} // wait until engine is running
}