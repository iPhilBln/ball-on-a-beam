#include "HC_SR04.h"

HC_SR04& HC_SR04::getInstance(uint8_t ball_radius) {
    static HC_SR04 _instance;

    if (_instanceUltrasonicCreated == false) {
        _instance.setBallRadius(ball_radius);
        _instance.setTemperature();

        _instanceUltrasonicCreated = true;
    }
    return _instance;
}

bool    HC_SR04::setOffset(void) {
    uint16_t  counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    uint16_t      timeout   = 10000U;
    unsigned long timestamp = millis();

    while (counter < 250U) {
        if (millis() - timestamp > timeout) {
            Serial.println("Initialization for ultrasonic sensor failed!");
            return false;
        }

        unsigned long cycle_time = millis() + 10U; // 10ms per measurement cycle

        distance = getDistancePure();
        (0U < distance && distance < 150U) && (distance_old - 1 <= distance || distance <= distance_old + 1) ? counter++ : counter = 0;
        distance_old = distance;
        while (millis() < cycle_time) {}
    }

    _offset = static_cast<uint8_t>(distance);
    _distance = _ball_radius;
    _echo_counter = 0U;

    Serial.println("Temperature: " + String(_temperature));
    Serial.println("Velocity: " + String(_ultrasonic_speed));
    Serial.println("Offset ultrasonic sensor: " + String(_offset));
    return true;
}

void    HC_SR04::setUltrasonicSpeed(double temperature) {
    _ultrasonic_speed = sqrt(1.402F * 8.3145F * (273.15F + temperature) / 0.02896F);
}

void HC_SR04::setupSensor(void) {
    stopTimerEcho();

    // Timer/Counter Control Registers
    TCCR4A = 0;
    TCCR4B = _BV(ICNC4) | _BV(ICES4); // Input Capture Noise Canceler and rising edge
    TCCR4C = 0;

    // Timer/Counter Interrupt Mask Register
    TIMSK4 = _BV(ICIE4) | _BV(TOIE4); // Input Capture and Timer Overflow Interrupt

    _SET_GPIO_DIRECTION_ULTRASONIC_ECHO();
    _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER();

    _measurement_failed = 0;
}

void   HC_SR04::startTimerEcho(void){
    _echo_counter = 0;
    _echo_timeout = false;

    TCNT4   = 0;            // reset counter value timer 4
    TCCR4B |= _BV(ICES4);   // capture ICR4 on rising edge at first
    TCCR4B |= _BV(CS40);    // prescaler = 1


    _measurement_completed = false;

    _ULTRASONIC_TRIGGER_START();
}

void   HC_SR04::stopTimerEcho(void){
    TCCR4B &= ~( _BV(CS42) | _BV(CS41) | _BV(CS40) );
    TCNT4   = 0;
    _ULTRASONIC_TRIGGER_STOP();

    _echo_timeout = true;
    if (_measurement_completed == false) {
        _echo_counter = 0;
    }
}

void HC_SR04::isrEchoCapture(void) {
    if (_instanceUltrasonicCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();

        if (_instance._measurement_completed == false) {
            if (TCCR4B & _BV(ICES4)) {
                _instance._echo_counter = ICR4;                                 // capture rising edge
            }
            else {
                _instance._echo_counter = ICR4 - _instance._echo_counter;       // capture falling edge
                if (_instance._echo_timeout == false) _instance._measurement_completed = true;
            }

            TCCR4B &= ~( _BV(ICES4) );
        }
    }
}

void HC_SR04::isrEchoTimeout(void) {
    if (_instanceUltrasonicCreated) {
        HC_SR04& _instance = HC_SR04::getInstance();
        _instance.stopTimerEcho();
    }
}

/*      SETTER      */

void    HC_SR04::setBallRadius(uint8_t ball_radius) {
    _ball_radius = ball_radius;
}

void    HC_SR04::setTemperature(void) {
    float voltage       = (analogRead(A0) >> 2) / 255.0F * 5.0F; 
    float temperature   = (voltage - 0.75F) / 0.01F + 25.0F;
    float alpha         = 0.05F;

    _temperature = (1.0F - alpha) * _temperature + alpha * temperature;
    setUltrasonicSpeed(_temperature);
   
    //Serial.println("Temperature: " + String(analogRead(A0) >> 2));
    //Serial.println("Temperature: " + String(_temperature) + " -> Ultrasonic speed: " + String(_ultrasonic_speed)); 
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

uint8_t HC_SR04::getOffset(void) const {
    return _offset;
}

uint16_t HC_SR04::getDistancePure(void) {
    if (_ULTRASONIC_CHECK_LEVEL_ECHO()) return 0;
    
    setTemperature();

    startTimerEcho();

    while (_echo_timeout == false) {}
    
    uint32_t distance_local;
    distance_local  = _echo_counter;
    distance_local *= static_cast<uint16_t>(_ultrasonic_speed); 
    distance_local /= 32000U;

    return static_cast<uint16_t>(distance_local);
}

/*      FUNCTIONALITY       */

uint16_t HC_SR04::getDistance(void) {
    unsigned long delaySensor = millis() + 10U; 
    uint16_t distance_local   = getDistancePure();

    while (millis() < delaySensor) {};

    if (distance_local >= _offset) {
        distance_local   = _ball_radius + (distance_local - _offset);
        (_ball_radius <= distance_local && distance_local <= 450U - _ball_radius) ? _distance = distance_local : _distance;
    }
    else {
        return 500;
    }

    return _distance;
}

float HC_SR04::getDistanceSimulink(void) {
    static bool    measurement_start  = true;

    if (measurement_start) {         // 0ms
        if (_ULTRASONIC_CHECK_LEVEL_ECHO()) {
            measurement_start = true; 
            stopTimerEcho();

            if (_measurement_failed < 100) {
                _measurement_failed++;
            }
            else return 0xFFFF;

            return _distance / 1000.0F;
        }

        if (_echo_counter != 0) {
            _measurement_failed = 0;

            uint32_t distance_local;
            distance_local  = _echo_counter;
            distance_local *= static_cast<uint16_t>(_ultrasonic_speed);  
            distance_local /= 32000U;


            if (distance_local >= _offset) {
                distance_local  = _ball_radius + (distance_local - _offset);
                if (_ball_radius <= distance_local && distance_local <= 450U - _ball_radius) {
                    _distance = static_cast<uint16_t>(distance_local);
                };
            }
        }
        else {
            if (_measurement_failed < 100) {
                _measurement_failed++;
            }
            else return 0xFFFF;
        }

        startTimerEcho();
    }

    measurement_start = !measurement_start;

    return _distance / 1000.0F;
}

bool HC_SR04::beginUltrasonic(void) {
    Serial.println("Begin ultrasonic sensor calibration.");
    
    for (uint8_t i = 0; i < 100; i++) setTemperature();

    setupSensor();
    return setOffset();
}