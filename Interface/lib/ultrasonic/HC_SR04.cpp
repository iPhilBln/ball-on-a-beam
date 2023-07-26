#include "HC_SR04.h"



//volatile uint16_t HC_SR04::_echo_counter = 0;
//volatile bool     HC_SR04::_echo_timeout = false;
/*
HC_SR04::HC_SR04() {
    setBallRadius(20);
    setTemperature(20.0);
    setTimerPrescaler(1);
}

HC_SR04::HC_SR04(uint8_t ball_radius, uint8_t timer_prescaler) {
    setBallRadius(ball_radius);
    setTemperature(20.0);
    switch (timer_prescaler) {
        case 1 : setTimerPrescaler(1);   break;
        case 8 : setTimerPrescaler(8);   break;
        default: setTimerPrescaler(1);   Serial.println("Unkown Prescaler! New Prescaler für both timers is 1.");   break;
    }
}

HC_SR04::HC_SR04(double temperature, uint8_t timer_prescaler) {
    setBallRadius(20);
    setTemperature(temperature);
    switch (timer_prescaler) {
        case 1 : setTimerPrescaler(1);   break;
        case 8 : setTimerPrescaler(8);   break;
        default: setTimerPrescaler(1);   Serial.println("Unkown Prescaler! New Prescaler für both timers is 1.");   break;
    }
}

HC_SR04::HC_SR04(uint8_t ball_radius, double temperature, uint8_t timer_prescaler) {
    setBallRadius(ball_radius);
    setTemperature(temperature);
    switch (timer_prescaler) {
        case 1 : setTimerPrescaler(1);   break;
        case 8 : setTimerPrescaler(8);   break;
        default: setTimerPrescaler(1);   Serial.println("Unkown Prescaler! New Prescaler für both timers is 1.");   break;
    }
}
*/

HC_SR04& HC_SR04::getInstance(uint8_t ball_radius = 20, double  temperature, uint8_t timer_prescaler = 1) {
    static HC_SR04 _instance;

    if (_instanceCreated == false) {
        _instance.setBallRadius(ball_radius);
        _instance.setTemperature(temperature);
        _instance.setTimerPrescaler(timer_prescaler);
        //_instance.beginUltrasonic();

        _instanceCreated = true;
    }
    return _instance;
}

void    HC_SR04::setTimerPrescaler(uint8_t timer_prescaler) {
    _timer_prescaler = timer_prescaler;
}

uint8_t HC_SR04::getTimerPrescaler(void) {
    return _timer_prescaler;
}

void    HC_SR04::setUltrasonicSpeed(double temperature) {
    _ultrasonic_speed = sqrt(1.402 * 8.3145 * (273.15 + temperature) / 0.02896);
}

double  HC_SR04::getUltrasonicSpeed(void) {
    return _ultrasonic_speed;
}

void    HC_SR04::setBallRadius(uint8_t ball_radius) {
    _ball_radius = ball_radius;
}

uint8_t HC_SR04::getBallRadius(void) {
    return _ball_radius;
}

void    HC_SR04::setTemperature(double temperature) {
    // hier noch Sensor implementieren 
    _temperature = temperature;
    setUltrasonicSpeed(_temperature); 
}

double  HC_SR04::getTemperature(void) {
    return _temperature;
}

void    HC_SR04::setOffsetLeft(void) {
    uint8_t   counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    while (counter < 250) {
        unsigned long cycle_time = micros() + 5500; // 5,5ms per measurement cycle

        distance = (getDistanceSensorLeft() - 0 - _ball_radius);
        (0 < distance && distance < 150) && (distance_old - 1 <= distance || distance <= distance_old + 1) ? counter++ : counter = 0;
        distance_old = distance;
        while (micros() < cycle_time) {}
    }
    _offset_left = static_cast<uint8_t>(distance);

    Serial.println("Offset left: " + String(_offset_left));
}

uint8_t HC_SR04::getOffsetLeft(void) {
    return _offset_left;
}

void    HC_SR04::setOffsetRight(void) {
    uint8_t   counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    while (counter < 250) {
        unsigned long cycle_time = micros() + 5500; // 5,5ms per measurement cycle

        distance = 450 - _ball_radius - getDistanceSensorRight();
        (0 < distance && distance < 150) && (distance_old - 1 <= distance || distance <= distance_old + 1) ? counter++ : counter = 0;
        distance_old = distance;
        while (micros() < cycle_time) {}
    }
    _offset_right = static_cast<uint8_t>(distance);

    Serial.println("Offset right: " + String(_offset_right));
}

uint8_t HC_SR04::getOffsetRight(void) {
    return _offset_right;
}

uint16_t HC_SR04::getDistanceSensorLeft(void) {
    if (_ULTRASONIC_CHECK_LEVEL_ECHO_LEFT()) return _distance_left;

             _echo_timeout       = false;
             _echo_counter_left  = 0;
    uint16_t echo_counter_local  = 0;
    uint32_t distance_local      = 0;

    startTimerEchoLeft();
    _ULTRASONIC_TRIGGER_LEFT_START();

    while (_echo_counter_left == 0) {
        if (_echo_timeout) {
            stopTimerEchoLeft();
            _ULTRASONIC_TRIGGER_LEFT_STOP();
            //Serial.println("Timeout left rising edge!");
            return _distance_left;
        }
    }

    echo_counter_local = _echo_counter_left;
    _echo_counter_left = 0;

    while (_echo_counter_left == 0) {
        if (_echo_timeout) {
            stopTimerEchoLeft();
            _ULTRASONIC_TRIGGER_LEFT_STOP();
            //Serial.println("Timeout left falling edge!");
            return _distance_left;
        }
    }
    stopTimerEchoLeft();
    _ULTRASONIC_TRIGGER_LEFT_STOP();
    
    distance_local  = _echo_counter_left - echo_counter_local;
    distance_local *= _ultrasonic_speed; 
    distance_local /= (32000 / _timer_prescaler);
    distance_local  = 0 + _ball_radius + (distance_local - _offset_left);

    return (distance_local <= 450) ? _distance_left = static_cast<uint16_t>(distance_local) : _distance_left; 
}

uint16_t HC_SR04::getDistanceSensorRight(void) {
    if (_ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT()) return _distance_right;

             _echo_timeout       = false;
             _echo_counter_right = 0;
    uint16_t echo_counter_local  = 0;
    uint32_t distance_local      = 0;

    startTimerEchoRight();
    _ULTRASONIC_TRIGGER_RIGHT_START();

    while (_echo_counter_right == 0) {
        if (_echo_timeout) {
            stopTimerEchoLeft();
            _ULTRASONIC_TRIGGER_RIGHT_STOP();
            //Serial.println("Timeout right rising edge!");
            return _distance_right;
        }
    }

    echo_counter_local = _echo_counter_right;
    _echo_counter_right      = 0;

    while (_echo_counter_right == 0) {
        if (_echo_timeout) {
            stopTimerEchoRight();
            _ULTRASONIC_TRIGGER_RIGHT_STOP();
            //Serial.println("Timeout right falling edge!");
            return _distance_right;
        }
    }

    stopTimerEchoRight();
    _ULTRASONIC_TRIGGER_RIGHT_STOP();

    distance_local  = _echo_counter_right - echo_counter_local;
    distance_local *= _ultrasonic_speed; 
    distance_local /= (32000 / _timer_prescaler);
    distance_local  = 450 - _ball_radius - (distance_local - _offset_right);

    return (distance_local <= 450) ? _distance_right = static_cast<uint16_t>(distance_local) : _distance_right; 
}

uint16_t HC_SR04::getDistance(void) {
    unsigned long delaySensor    = micros() + 5500;
    uint16_t      distance_l     = 0;
    uint16_t      distance_r     = 0;
    uint16_t      distance_local = 0;
    
    distance_l = getDistanceSensorLeft();
    while (micros() < delaySensor) {};
    distance_r = getDistanceSensorRight();

    if (distance_l < 71) {
        distance_local = distance_l;
    }
    else if (distance_r > 371) {
        distance_local = distance_r;
    }
    else if (270 < distance_l && distance_l < 371) {
        distance_local = distance_l;
    }
    else if (70 < distance_r && distance_r < 171) {
        distance_local = distance_r;
    }
    else if (170 < distance_l && distance_l < 226) {
        distance_local = distance_l;
    }
    else if (225 < distance_r && distance_r < 271) {
        distance_local = distance_r;
    }
    else {
        distance_local = (distance_l + distance_r) >> 1;
    }

    return distance_local < 450 - _ball_radius ? _distance = distance_local : _distance;
}

void HC_SR04::beginSensorLeft(void) {
    stopTimerEchoLeft();

    TCCR4A = 0;
    TCCR4B = _BV(ICNC4) | _BV(ICES4) | _BV(WGM42);
    TCCR4C = 0;
    TIMSK4 = _BV(ICIE4) | _BV(OCIE4A);

    switch (_timer_prescaler) {
        case 1 : OCR4A = static_cast<uint16_t>(3.5 * 16000 - 1);   break;
        case 8 : OCR4A = static_cast<uint16_t>(3.5 * 8000 - 1);    break;
    }

    _SET_GPIO_DIRECTION_ULTRASONIC_ECHO_LEFT();
    _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_LEFT();

    setOffsetLeft();
}

void   HC_SR04::startTimerEchoLeft(void){
    TCNT4   = 0;                      // reset counter value timer 4
    switch (_timer_prescaler) {
        case 1 : TCCR4B |= _BV(ICES4) | _BV(CS40);              break;
        case 8 : TCCR4B |= _BV(ICES4) | _BV(CS41) | _BV(CS40);  break;
    }
}

void   HC_SR04::stopTimerEchoLeft(void){
    TCCR4B &= ~( _BV(CS42) | _BV(CS41) | _BV(CS40) );
}

void HC_SR04::isrEchoLeftCapture(void) {
    /*
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        _instance._echo_counter_left = TCNT4;
        TCCR4B &= ~( _BV(ICES4) );
    }
    */
    _echo_counter_left = TCNT4;
    TCCR4B &= ~( _BV(ICES4) );
}

void HC_SR04::isrEchoLeftTimeout(void) {
    /*
    if (_instanceCreated) {
        Serial.println(2);
        HC_SR04& _instance = HC_SR04::getInstance();

        _instance._echo_timeout = true;
    }
    */
    _echo_timeout = true;
}


void HC_SR04::beginSensorRight(void) {
    stopTimerEchoRight();

    TCCR5A = 0;
    TCCR5B = _BV(ICNC5) | _BV(ICES5) | _BV(WGM52);
    TCCR5C = 0;
    TIMSK5 = _BV(ICIE5) | _BV(OCIE5A);

    switch (_timer_prescaler) {
        case 1 : OCR5A = static_cast<uint16_t>(3.5 * 16000 - 1);   break;
        case 8 : OCR5A = static_cast<uint16_t>(3.5 * 8000 - 1);    break;
    }

    _SET_GPIO_DIRECTION_ULTRASONIC_ECHO_RIGHT();
    _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_RIGHT();

    setOffsetRight();
}

void   HC_SR04::startTimerEchoRight(void){
    TCNT5   = 0;
    switch (_timer_prescaler) {
        case 1 : TCCR5B |= _BV(ICES5) | _BV(CS50);              break;
        case 8 : TCCR5B |= _BV(ICES5) | _BV(CS51) | _BV(CS50);  break;
    }
}

void   HC_SR04::stopTimerEchoRight(void){
    TCCR5B &= ~( _BV(CS52) | _BV(CS51) | _BV(CS50) );
}

void HC_SR04::isrEchoRightCapture(void) {
    /*
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        _instance._echo_counter_right = TCNT5;
        TCCR5B &= ~( _BV(ICES5) );
    }
    */
    _echo_counter_right = TCNT5;
    TCCR5B &= ~( _BV(ICES5) );
}

void HC_SR04::isrEchoRightTimeout(void) {
    /*
    if (_instanceCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        _instance._echo_timeout = true;
    }
    */
    _echo_timeout = true;
}

void HC_SR04::beginUltrasonic(void) {
    stepper.setDegreeTarget(20.0);
    Serial.println("Begin left.");
    beginSensorLeft();
    stepper.setDegreeTarget(-20.0);
    Serial.println("Begin right.");
    beginSensorRight();
    stepper.setDegreeTarget(0.0);
    
    while(stepper.getState()) {} // wait until engine is running
}
