#include "HC_SR04.h"

HC_SR04& HC_SR04::getInstance(uint8_t ball_radius, double  temperature, uint8_t timer_prescaler) {
    static HC_SR04 _instance;

    if (_instanceUltrasonicCreated == false) {
        _instance.setBallRadius(ball_radius);
        _instance.setTemperature(temperature);
        _instance.setTimerPrescaler(timer_prescaler);

        _instanceUltrasonicCreated = true;
    }
    return _instance;
}

void    HC_SR04::setOffset(void) {
    uint16_t  counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    while (counter < 500U) {
        unsigned long cycle_time = millis() + 10U; // 10ms per measurement cycle

        distance = getDistancePure() - _ball_radius;
        (0U < distance && distance < 150U) && (distance_old - 1 <= distance || distance <= distance_old + 1) ? counter++ : counter = 0;
        distance_old = distance;
        while (millis() < cycle_time) {}
    }
    _offset = static_cast<uint8_t>(distance);

    Serial.println("Offset: " + String(_offset));
}

void    HC_SR04::setUltrasonicSpeed(double temperature) {
    _ultrasonic_speed = sqrt(1.402F * 8.3145F * (273.15F + temperature) / 0.02896F);
}

void HC_SR04::beginSensor(void) {
    stopTimerEcho();

    TCCR4A = 0;
    TCCR4B = _BV(ICNC4) | _BV(ICES4) | _BV(WGM42);
    TCCR4C = 0;
    TIMSK4 = _BV(ICIE4) | _BV(OCIE4A);

    switch (_timer_prescaler) {
        case 1 : OCR4A = static_cast<uint16_t>(3.5F * 16000 - 1);   break;  // echo time out after 3.5ms
        case 8 : OCR4A = static_cast<uint16_t>(3.5F * 8000 - 1);    break;  // echo time out after 3.5ms
    }

    _SET_GPIO_DIRECTION_ULTRASONIC_ECHO();
    _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER();

    setOffset();
}

void   HC_SR04::startTimerEcho(void){
    TCNT4   = 0;            // reset counter value timer 4
    TCCR4B |= _BV(ICES4);   // capture TCNT4 on rising edge at first
    
    switch (_timer_prescaler) {
        case 1 : TCCR4B |= _BV(CS40);              break;
        case 8 : TCCR4B |= _BV(CS41) | _BV(CS40);  break;
    }

    _echo_counter = 0;
    _echo_timeout = false;
    _measurement_completed = false;

    _ULTRASONIC_TRIGGER_START();
}

void   HC_SR04::stopTimerEcho(void){
    TCCR4B &= ~( _BV(CS42) | _BV(CS41) | _BV(CS40) );
    _ULTRASONIC_TRIGGER_STOP();
}

void HC_SR04::isrEchoCapture(void) {
    if (_instanceUltrasonicCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        if (TCCR4B & _BV(ICES4)) {
            _instance._echo_counter = TCNT4;                                 // capture rising edge
        }
        else {
            _instance._echo_counter = TCNT4 - _instance._echo_counter;       // capture falling edge
            _instance._measurement_completed = true;
        }

        TCCR4B &= ~( _BV(ICES4) );
    }
}

void HC_SR04::isrEchoTimeout(void) {
    if (_instanceUltrasonicCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        if (_instance._measurement_completed == false) _instance._echo_timeout = true;
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

uint8_t HC_SR04::getOffset(void) const {
    return _offset;
}

uint16_t HC_SR04::getDistancePure(void) {
    if (_ULTRASONIC_CHECK_LEVEL_ECHO()) return 0;

    startTimerEcho();

    while (_echo_timeout == false && _measurement_completed == false) {}

    stopTimerEcho();

    if (_measurement_completed == false) return 0;
    
    uint32_t distance_local;
    distance_local  = _echo_counter;
    //Serial.println(_echo_counter);
    distance_local *= _ultrasonic_speed; 
    distance_local /= (32000U / _timer_prescaler);
    distance_local  = _ball_radius + (distance_local - _offset);

    return static_cast<uint16_t>(distance_local);
}

uint16_t HC_SR04::getDistance(void) {
    unsigned long delaySensor = millis() + 10U; 
    uint16_t distance_local   = getDistancePure();

    while (millis() < delaySensor) {};

    (distance_local <= 450U - _ball_radius) ? _distance = 450U - distance_local : _distance;
    return _distance;
}

float HC_SR04::getDistanceSimulink(void) {
    static uint8_t counter = 0;

    if (counter == 0U) {         // 0ms
        if (_ULTRASONIC_CHECK_LEVEL_ECHO()) {
            counter = 0; 
            stopTimerEcho();
            return _distance / 1000.0F;
        }

        stopTimerEcho();

        if (_echo_counter > 0 && _echo_timeout == false) {
            uint32_t distance_local;
            distance_local  = _echo_counter;
            Serial.println(_echo_counter);
            distance_local *= _ultrasonic_speed; 
            distance_local /= (32000U / _timer_prescaler);
            //Serial.print(distance_local);
            distance_local  = 450U - _ball_radius - (distance_local - _offset);

            (distance_local <= 450U - _ball_radius) ? _distance = static_cast<uint16_t>(distance_local) : _distance; 
            //Serial.println('\t' + String(_distance) + '\t' + String(_offset));
        }

        startTimerEcho();
    }

    counter < 4U ? counter++ : counter = 0U;  // count 4 times -> 10ms measurement cycle
    return _distance / 1000.0F;
}

bool HC_SR04::beginUltrasonic(void) {
    stepper.setDegreeTarget(-20.0F);
    Serial.println("Begin sensor calibration.");
    beginSensor();
    stepper.setDegreeTarget(0.0F);
    
    _distance = 0U;
    _echo_counter = 0U;
    
    while(stepper.getState()) {} // wait until engine is running
    return true;
}