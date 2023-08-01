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

void CONTROLLER::setTimer(void) {

    TCCR2A = 0;
    TCCR2B = 0;

    TCCR2A |= _BV(WGM21);   // enable Clear Timer on Compare immedately 
    TIMSK2 |= _BV(OCIE2A);  // enable timer 2 compare interrupt for OCR2A

    if (_sensor == SENSOR::CAPAITY) {

    }
    else if (_sensor == SENSOR::RESISTOR) {

    }
    else if (_sensor == SENSOR::TOF) {

    }
    else if (_sensor == SENSOR::ULTRASONIC) {
        // 11ms cycle time
        OCR2A   = 171;
        TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20); // prescaler 1024

    }
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
}

void CONTROLLER::stop(void) {
    stepper.stop();
}

void CONTROLLER::runPcontroller(void) {
    switch (_sensor) {
        case SENSOR::CAPAITY :    break;
        case SENSOR::RESISTOR :   break;
        case SENSOR::TOF :        break;
        case SENSOR::ULTRASONIC : break;//ultrasonicPtr = &HC_SR04::getInstance(); break;
        case SENSOR::undefined :  break;
    }

    setTimer();

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
    setTimer();

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

    setTimer();
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
    setTimer();
    static float freq = 0.0;
    
    while (true) {
        _running = true;

        STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance();
        
        float freq_temp = recSimu();
        isnan(freq_temp) ? freq = freq : (-151 < freq_temp && freq_temp < 151 ? freq = freq_temp : freq = freq);

        stepper.setFreq(freq);

        FLOATUNION_t val;
        val.number[0] = stepper.getFreq();
        val.number[1] = stepper.getDegreeActual();
        val.number[2] = 0.0;
        val.number[3] = 0.0;
        
        //transmitFloatToSimulink(degree);
        //transmitFloatToSimulink(rpm);

        sendSimu(val);

        while(_running) {}
    }
    Serial.println("Ende");
}