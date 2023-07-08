#include "STEPPER_ENGINE.h"

/*      CONSTRUCTORS        */

STEPPER_ENGINE& STEPPER_ENGINE::getInstance(ENGINE_STEP_MODE step_mode) {
    static STEPPER_ENGINE _instance;

    _instance.setDegreeTargetMax(45.0);
    _instance.setDegreeTarget(0.0);
    _instance.setRpmMin(30);
    _instance.setRpmMax(200);
    _instance.setStepMode(step_mode);
    _instance.setStepsPerRevolution(400); 

    _instanceCreated = true;
    return _instance;
}

STEPPER_ENGINE& STEPPER_ENGINE::getInstance(uint16_t steps_per_revolution, ENGINE_STEP_MODE step_mode) {
    static STEPPER_ENGINE _instance;

    _instance.setDegreeTargetMax(45.0);
    _instance.setDegreeTarget(0.0);
    _instance.setRpmMin(30);
    _instance.setRpmMax(200);
    _instance.setStepMode(step_mode);
    _instance.setStepsPerRevolution(steps_per_revolution);

    _instanceCreated = true;
    return _instance;  
}

STEPPER_ENGINE& STEPPER_ENGINE::getInstance(uint16_t rpm_min, uint16_t rpm_max, uint16_t steps_per_revolution, ENGINE_STEP_MODE step_mode) {
    static STEPPER_ENGINE _instance;

    _instance.setDegreeTargetMax(45.0);
    _instance.setDegreeTarget(0.0);
    _instance.setRpmMin(rpm_min);
    _instance.setRpmMax(rpm_max);
    _instance.setStepMode(step_mode);
    _instance.setStepsPerRevolution(steps_per_revolution);

    _instanceCreated = true;
    return _instance;     
}

STEPPER_ENGINE& STEPPER_ENGINE::getInstanceIsr(void) {
    static STEPPER_ENGINE _instance;
    return _instance;
}


/*      PRIVATE METHODS      */

void       STEPPER_ENGINE::setDegreeActual(int32_t degree_actual) {
    _degree_actual += degree_actual; 
}

void        STEPPER_ENGINE::setRpmActual(void) {
    static ENGINE_DIRECTION direction_local = ENGINE_DIRECTION::undefined;

    if (_rpm_actual <= _rpm_max) {
        if (_direction == direction_local) {
            _rpm_counter++;
        }
        else {
            _rpm_counter = 0;
            direction_local = _direction;
        }
        
        _rpm_actual = _rpm_min + 3 * _rpm_counter;

        uint32_t time = 60 * F_CPU / _rpm_actual / _steps_per_revolution / _timer_prescaler - 1;
        OCR1A = static_cast<uint16_t>(time);
    }
}

void        STEPPER_ENGINE::setDegreePerStep(uint16_t degree_per_step) {
    switch (_step_mode) {
        case ENGINE_STEP_MODE::halfstep : _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / degree_per_step);        break;
        case ENGINE_STEP_MODE::fullstep : _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / degree_per_step * 2.0);  break;
    }    
}

void        STEPPER_ENGINE::setStepMode(ENGINE_STEP_MODE step_mode) {
    _step_mode = step_mode;
}

void        STEPPER_ENGINE::setDirection(ENGINE_DIRECTION direction) {
    _direction = direction;
}

void        STEPPER_ENGINE::move(void) {
    switch (_step_mode) {
        case ENGINE_STEP_MODE::halfstep:
        
            switch (_direction) {
                case ENGINE_DIRECTION::ccw: _step_counter > 0 ? _step_counter-- : _step_counter = 7;    break;
                case ENGINE_DIRECTION::cw : _step_counter < 7 ? _step_counter++ : _step_counter = 0;    break;
            }
            
            switch (_step_counter) {
                case 0: H_L1CW_L2CCW();  break; 
                case 1: H_L1CW_L2OFF();  break; 
                case 2: H_L1CW_L2CW();   break;   
                case 3: H_L1OFF_L2CW();  break; 
                case 4: H_L1CCW_L2CW();  break; 
                case 5: H_L1CCW_L2OFF(); break;    
                case 6: H_L1CCW_L2CCW(); break;    
                case 7: H_L1OFF_L2CCW(); break;    
            }
            break;

        case ENGINE_STEP_MODE::fullstep:

            switch (_direction) {
                case ENGINE_DIRECTION::ccw: _step_counter > 0 ? _step_counter-- : _step_counter = 3;    break;
                case ENGINE_DIRECTION::cw : _step_counter < 3 ? _step_counter++ : _step_counter = 0;    break;
            }
            
            switch (_step_counter) {
                case 0: F_L1CW_L2CCW();  break; 
                case 1: F_L1CW_L2CW();   break; 
                case 2: F_L1CCW_L2CW();  break;   
                case 3: F_L1CCW_L2CCW(); break;  
            }     
            break;                
    }
}  

void        STEPPER_ENGINE::startTimerEngine(void) {
    TCNT1 = 0;

    switch (_timer_prescaler)                                       
    {
        case 1    :  TCCR1B |= _BV(CS10);              break;  // SET CS12 & CS11 & CS10 bit for prescaler 1
        case 8    :  TCCR1B |= _BV(CS11);              break;  // SET CS12 & CS11 & CS10 bit for prescaler 8  
        case 64   :  TCCR1B |= _BV(CS11) | _BV(CS10);  break;  // SET CS12 & CS11 & CS10 bit for prescaler 64
        case 256  :  TCCR1B |= _BV(CS12);              break;  // SET CS12 & CS11 & CS10 bit for prescaler 256
        case 1024 :  TCCR1B |= _BV(CS12) | _BV(CS10);  break;  // SET CS12 & CS11 & CS10 bit for prescaler 1024
        default   :  TCCR1B |= _BV(CS11);              break;  // SET CS12 & CS11 & CS10 bit for prescaler 8 
    }
}

void        STEPPER_ENGINE::stopTimerEngine(void) {
    TCCR1B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

void        STEPPER_ENGINE::isrSetEngine(void) {
    if (_instanceCreated) {
        STEPPER_ENGINE& _instance = STEPPER_ENGINE::getInstanceIsr();
        
        float test= _instance.getDegreeTargetMax();
        Serial.println("Test:" + String(test));
        Serial.print("I:");
        Serial.println(_instance._degree_target);

        if (_instance._degree_target  > (_instance._degree_actual + _instance._degree_per_step / 2)) {
            _instance.setDegreeActual(_instance._degree_per_step);
            _instance.setRpmActual();
            _instance.setDirection(ENGINE_DIRECTION::ccw);
            _instance.move();
        }
        else if (_instance._degree_target  < (_instance._degree_actual - _instance._degree_per_step / 2)) {
            _instance.setDegreeActual(-1 * _instance._degree_per_step);
            _instance.setRpmActual();
            _instance.setDirection(ENGINE_DIRECTION::cw);
            _instance.move();
        }
        else {
            _instance.stopTimerEngine();
        } 
    }
}


/*      PUBLIC METHODS      */

// SETTER

void       STEPPER_ENGINE::setDegreeTargetMax(float degree_target_max) {
    _degree_target_max = static_cast<int32_t>(degree_target_max * 1000);
}

void        STEPPER_ENGINE::setDegreeTarget(float degree_target, bool reset) {
    if (reset) {
        _degree_target = 0;
        return;
    }

    int32_t degree_target_local = static_cast<int32_t>(degree_target * 1000);

    if (degree_target_local > _degree_target_max)           _degree_target = _degree_target_max;
    else if (degree_target_local < -1 * _degree_target_max) _degree_target = -1 * _degree_target_max;
    else                                                    _degree_target = degree_target;
    Serial.println("F: " + String(degree_target_local));
    //startTimerEngine();
}

void        STEPPER_ENGINE::setRpmMin(uint16_t rpm_min) {
    stopTimerEngine();
    _rpm_min = rpm_min;
    begin();
}

void        STEPPER_ENGINE::setRpmMax(uint16_t rpm_max) {
    _rpm_max = rpm_max;
}

void        STEPPER_ENGINE::setStepsPerRevolution(uint16_t steps_per_revolution) {
    _steps_per_revolution = steps_per_revolution;
    setDegreePerStep(_steps_per_revolution);
}


// GETTER

float      STEPPER_ENGINE::getDegreeActual(void) const {
    return static_cast<float>(_degree_actual / 1000.0);   
}

float      STEPPER_ENGINE::getDegreeTarget(void) const {
    Serial.println("R: " + String(_degree_target));
    return static_cast<float>(_degree_target / 1000.0);
}

float      STEPPER_ENGINE::getDegreeTargetMax(void) const {
    return static_cast<float>(_degree_target_max / 1000.0);
}

uint16_t    STEPPER_ENGINE::getRpmActual(void) const {
    return _rpm_actual;
}

uint16_t    STEPPER_ENGINE::getRpmMin(void) const {
    return _rpm_min;
}

uint16_t    STEPPER_ENGINE::getRpmMax(void) const {
    return _rpm_max;
}

uint16_t    STEPPER_ENGINE::getStepsPerRevolution(void) const {
    return _steps_per_revolution;
}

float       STEPPER_ENGINE::getDegreePerStep(void) const {
    return static_cast<float>(_degree_per_step / 1000.0);
}

ENGINE_STEP_MODE        STEPPER_ENGINE::getStepMode(void) const {
    return _step_mode;
}

void        STEPPER_ENGINE::begin(void) {

    // turn on CTC mode - Clear Timer on Compare 
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // set entire TCCR1B register to 0

    TCCR1B = _BV(WGM12);   // enable Clear Timer on Compare immedately for OCR1A
    TIMSK1 = _BV(OCIE1A);  // enable timer 1 compare interrupt

    uint16_t arr_prescaler[] = {1, 8, 64, 256, 1024};
    uint8_t  length = sizeof(arr_prescaler) / sizeof(arr_prescaler[0]);

    for (uint8_t i = 0; i < length; i++) {
        uint32_t prescaler_helper = 60 * F_CPU / _rpm_min / _steps_per_revolution / arr_prescaler[i] - 1;
        if (prescaler_helper < 0xFFFF) {
            _timer_prescaler = arr_prescaler[i];
            break;
        }
    }

    uint32_t time = 60 * F_CPU / _rpm_min / _steps_per_revolution / _timer_prescaler - 1;
    OCR1A = static_cast<uint16_t>(time);

    //setDegreeTarget(45.0);
}

void        STEPPER_ENGINE::stop(void) {
    stopTimerEngine();
}

