#include "Arduino.h"
#include "STEPPER_ENGINE.h"

// Initialisierung des statischen Members außerhalb der Klasse
STEPPER_ENGINE* STEPPER_ENGINE::isrInstance = nullptr;

STEPPER_ENGINE::STEPPER_ENGINE(bool step_mode) {
    setDegreeTargetMax(45.0);
    setDegreeTarget(0.0);
    setRpmMin(30);
    setRpmMax(200);
    setStepMode(step_mode);
    setStepsPerRevolution(400); 
}

STEPPER_ENGINE::STEPPER_ENGINE(uint16_t steps_per_revolution, bool step_mode) {
    setDegreeTargetMax(45.0);
    setDegreeTarget(0.0);
    setRpmMin(30);
    setRpmMax(200);
    setStepMode(step_mode);
    setStepsPerRevolution(steps_per_revolution);  
}

STEPPER_ENGINE::STEPPER_ENGINE(uint16_t rpm_min, uint16_t rpm_max, uint16_t steps_per_revolution, bool step_mode) {
    setDegreeTargetMax(45.0);
    setDegreeTarget(0.0);
    setRpmMin(rpm_min);
    setRpmMax(rpm_max);
    setStepMode(step_mode);
    setStepsPerRevolution(steps_per_revolution);     
    
}

void       STEPPER_ENGINE::setDegreeActual(int32_t degree_actual) {
    _degree_actual += degree_actual; 
}

float      STEPPER_ENGINE::getDegreeActual(void) {
    return static_cast<float>(_degree_actual / 1000.0);   
}

void       STEPPER_ENGINE::setDegreeTargetMax(float degree_target_max) {
    _degree_target_max = static_cast<int32_t>(degree_target_max * 1000);
}

float      STEPPER_ENGINE::getDegreeTargetMax(void) {
    return static_cast<float>(_degree_target_max / 1000.0);
}

void        STEPPER_ENGINE::setDegreeTarget(float degree_target) {
    int32_t degree_target_local = static_cast<int32_t>(degree_target * 1000);

    if (degree_target_local > _degree_target_max)           _degree_target = _degree_target_max;
    else if (degree_target_local < -1 * _degree_target_max) _degree_target = -1 * _degree_target_max;
    else                                                    _degree_target = degree_target;
    Serial.println("F: " + String(degree_target_local));
    //startTimerEngine();
}

float      STEPPER_ENGINE::getDegreeTarget(void) {
    Serial.println("R: " + String(_degree_target));
    return static_cast<float>(_degree_target / 1000.0);
}

void        STEPPER_ENGINE::setRpmActual(void) {
    static ENGINE_DIRECTION direction_helper_local = ENGINE_DIRECTION::undefined;

    if (_direction_helper != direction_helper_local) {
        _direction_helper = direction_helper_local;
        _rpm_counter = 0;
        uint32_t time = 60 * F_CPU / _rpm_min / _steps_per_revolution / _timer_prescaler - 1;
        OCR1A = static_cast<uint16_t>(time);
    }
    else if (_rpm_actual < _rpm_max) {
        _rpm_counter++;
        uint32_t time = 60 * F_CPU / (_rpm_min + 3 * _rpm_counter) / _steps_per_revolution / _timer_prescaler - 1;
        OCR1A = static_cast<uint16_t>(time);
    }
}

uint16_t    STEPPER_ENGINE::getRpmActual(void) {
    return _rpm_actual;
}

void        STEPPER_ENGINE::setRpmMin(uint16_t rpm_min) {
    stopTimerEngine();
    _rpm_min = rpm_min;
    begin();
}

uint16_t    STEPPER_ENGINE::getRpmMin(void) {
    return _rpm_min;
}

void        STEPPER_ENGINE::setRpmMax(uint16_t rpm_max) {
    _rpm_max = rpm_max;
}

uint16_t    STEPPER_ENGINE::getRpmMax(void) {
    return _rpm_max;
}

void        STEPPER_ENGINE::setStepsPerRevolution(uint16_t steps_per_revolution) {
    _steps_per_revolution = steps_per_revolution;
    setDegreePerStep(_steps_per_revolution);
}

uint16_t    STEPPER_ENGINE::getStepsPerRevolution(void) {
    return _steps_per_revolution;
}

void        STEPPER_ENGINE::setDegreePerStep(uint16_t degree_per_step) {
    switch (_step_mode) {
        case true : _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / degree_per_step);        break;
        case false: _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / degree_per_step * 2.0);  break;
    }    
}

float       STEPPER_ENGINE::getDegreePerStep(void) {
    return static_cast<float>(_degree_per_step / 1000.0);
}

void        STEPPER_ENGINE::setStepMode(bool step_mode) {
    _step_mode = step_mode;
}

bool        STEPPER_ENGINE::getStepMode(void) {
    return _step_mode;
}

void        STEPPER_ENGINE::isrSetDegreeActual(void) {
    if (isrInstance != nullptr) {
        float test= isrInstance->getDegreeTargetMax();
        Serial.println("Test:" + String(test));
        Serial.print("I:");
        Serial.println(isrInstance->_degree_target);
        if (isrInstance->_degree_target  > (isrInstance->_degree_actual + isrInstance->_degree_per_step / 2)) {
            isrInstance->setDegreeActual(isrInstance->_degree_per_step);
            isrInstance->setRpmActual();
            isrInstance->turn_ccw(); 
        }
        else if (isrInstance->_degree_target  < (isrInstance->_degree_actual - isrInstance->_degree_per_step / 2)) {
            isrInstance->setDegreeActual(-1 * isrInstance->_degree_per_step);
            isrInstance->setRpmActual();
            isrInstance->turn_cw(); 
        }
        else {
            isrInstance->stopTimerEngine();
        }
    }       
}

void        STEPPER_ENGINE::begin(void) {
    
    isrInstance = this; // Zeiger auf die aktuelle Instanz setzen

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

void        STEPPER_ENGINE::turn_cw(void) {
    switch (_step_mode) {
        case true :
            _step_counter < 7 ? _step_counter : _step_counter = 0;
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

        case false:
            _step_counter < 3 ? _step_counter++ : _step_counter = 0;
            switch (_step_counter) {
                case 0: F_L1CW_L2CCW();  break; 
                case 1: F_L1CW_L2CW();   break; 
                case 2: F_L1CCW_L2CW();  break;   
                case 3: F_L1CCW_L2CCW(); break;  
            }                     
    }
    _direction_helper = ENGINE_DIRECTION::cw;
}

void        STEPPER_ENGINE::turn_ccw(void) {
    switch (_step_mode) {
        case true :
            _step_counter > 0 ? _step_counter-- : _step_counter = 7;
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

        case false:
            _step_counter > 0 ? _step_counter-- : _step_counter = 3;
            switch (_step_counter) {
                case 0: F_L1CW_L2CCW();  break; 
                case 1: F_L1CW_L2CW();   break; 
                case 2: F_L1CCW_L2CW();  break;   
                case 3: F_L1CCW_L2CCW(); break;  
            }                     
    }
    _direction_helper = ENGINE_DIRECTION::ccw;
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