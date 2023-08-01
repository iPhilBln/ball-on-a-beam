#include "STEPPER_ENGINE.h"
#include "avr8-stub.h"
#include "app_api.h"  //only needed with flash breakpoints

/*      CONSTRUCTORS        */

STEPPER_ENGINE& STEPPER_ENGINE::getInstance(ENGINE_STEP_MODE step_mode, uint16_t rpm_min, uint16_t rpm_max, uint16_t steps_per_revolution) {
    static STEPPER_ENGINE _instance;

    if (_instanceCreated == false) {
        _instance.setPosition(true);
        _instance.setDegreeTargetMax(45.0);
        _instance.setDegreeTarget(0.0);
        _instance.setStepMode(step_mode);
        _instance.setStepsPerRevolution(steps_per_revolution);
        _instance.setRpmMax(rpm_max);
        _instance.setRpmMin(rpm_min);

        _instanceCreated = true;
    }
    return _instance;     
}

/*      PRIVATE METHODS      */

void       STEPPER_ENGINE::setDegreeActual(bool reset) {
    if (reset) {
        _degree_actual = 0;
        return;
    }

    switch (_direction) {
        case ENGINE_DIRECTION::ccw : _degree_actual += _degree_per_step;   break;
        case ENGINE_DIRECTION::cw  : _degree_actual -= _degree_per_step;   break;
        default: break;
    }
}

void        STEPPER_ENGINE::setRpmActual(void) {
    static ENGINE_DIRECTION direction_local = ENGINE_DIRECTION::undefined;

    if (_direction == direction_local) {
        if (_rpm_actual < _rpm_max) {
            _rpm_counter++;
        }
    }   
    else {
        _rpm_counter = 0;
        direction_local = _direction;
    }
    
    _rpm_actual = _rpm_min + 3 * _rpm_counter;

    setPrescaler(1.0 * _rpm_actual);
}

void        STEPPER_ENGINE::setDegreePerStep(uint16_t steps_per_revolution) {
    _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / steps_per_revolution);
    /*
    switch (_step_mode) {
        case ENGINE_STEP_MODE::halfstep : _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / steps_per_revolution);  break;
        case ENGINE_STEP_MODE::fullstep : _degree_per_step = static_cast<uint16_t>(360.0 * 1000.0 / steps_per_revolution);      break;
        default: break;
    }    
    */
}

void        STEPPER_ENGINE::setStepMode(ENGINE_STEP_MODE step_mode) {
    _step_mode = step_mode;
}

void        STEPPER_ENGINE::setDirection(ENGINE_DIRECTION direction) {
    _direction = direction;
}

void        STEPPER_ENGINE::setPrescaler(float freq) {
        uint16_t arr_prescaler[] = {1, 8, 64, 256, 1024};
        uint8_t  length = sizeof(arr_prescaler) / sizeof(arr_prescaler[0]);

        for (uint8_t i = 0; i < length; i++) {
            uint32_t ocr_helper = static_cast<uint32_t>(60.0 * F_CPU / freq / _steps_per_revolution / arr_prescaler[i] - 1.0);
            if (ocr_helper < 0xFFFF) {
                _timer_prescaler = arr_prescaler[i];
                setOCR(static_cast<uint16_t>(ocr_helper));
                break;
                /*
                if (_timer_prescaler != arr_prescaler[i]) {
                    stopTimerEngine();
                }
                else if (TCNT1 > freq_helper) {
                    stopTimerEngine();
                }
                
                setPrescaler(arr_prescaler[i]);
                setOCR(static_cast<uint16_t>(freq_helper));
                startTimerEngine();
                
                */
            }
        }
}

void        STEPPER_ENGINE::move(void) {
    switch (_step_mode) {
        case ENGINE_STEP_MODE::halfstep:
        
            switch (_direction) {
                case ENGINE_DIRECTION::ccw: _step_counter > 0 ? _step_counter-- : _step_counter = 7;    break;
                case ENGINE_DIRECTION::cw : _step_counter < 7 ? _step_counter++ : _step_counter = 0;    break;
                default: break;
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
                default: break;
            }
            break;

        case ENGINE_STEP_MODE::fullstep:

            switch (_direction) {
                case ENGINE_DIRECTION::ccw: _step_counter > 0 ? _step_counter-- : _step_counter = 3;    break;
                case ENGINE_DIRECTION::cw : _step_counter < 3 ? _step_counter++ : _step_counter = 0;    break;
                default: break;
            }
            
            switch (_step_counter) {
                case 0: F_L1CW_L2CCW();  break; 
                case 1: F_L1CW_L2CW();   break; 
                case 2: F_L1CCW_L2CW();  break;   
                case 3: F_L1CCW_L2CCW(); break;  
                default: break;
            }     
            break;                
    }
}  

void        STEPPER_ENGINE::startTimerEngine(void) {
    OCR1A = _ocr;

    switch (_timer_prescaler)                                       
    {
        case 1    :  TCCR1B |= _BV(CS10);             TCCR1B &= ~( _BV(CS11) | _BV(CS12) ); break;  // SET CS12 & CS11 & CS10 bit for prescaler 1
        case 8    :  TCCR1B |= _BV(CS11);             TCCR1B &= ~( _BV(CS10) | _BV(CS12) ); break;  // SET CS12 & CS11 & CS10 bit for prescaler 8  
        case 64   :  TCCR1B |= _BV(CS11) | _BV(CS10); TCCR1B &= ~( _BV(CS12) );             break;  // SET CS12 & CS11 & CS10 bit for prescaler 64
        case 256  :  TCCR1B |= _BV(CS12);             TCCR1B &= ~( _BV(CS10) | _BV(CS11) ); break;  // SET CS12 & CS11 & CS10 bit for prescaler 256
        case 1024 :  TCCR1B |= _BV(CS12) | _BV(CS10); TCCR1B &= ~( _BV(CS11) );             break;  // SET CS12 & CS11 & CS10 bit for prescaler 1024
        default   :  TCCR1B |= _BV(CS11);             TCCR1B &= ~( _BV(CS10) | _BV(CS12) ); break;  // SET CS12 & CS11 & CS10 bit for prescaler 8 
    }

    TCNT1 = 0;
}

void        STEPPER_ENGINE::stopTimerEngine(void) {
    TCNT1   = 0;
    TCCR1B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

void        STEPPER_ENGINE::isrMoveEngine(void) {
    if (STEPPER_ENGINE::_instanceCreated) {
        STEPPER_ENGINE& _instance = STEPPER_ENGINE::getInstance();
        if (_instance._setPosition == false) {
            _instance.setDegreeActual();
            _instance.move();
            _instance.startTimerEngine();
        }
        else {     
            if (_instance._degree_actual < (_instance._degree_target - _instance._degree_per_step / 2)) {
                _instance.setDirection(ENGINE_DIRECTION::ccw);
                _instance.setDegreeActual();
                _instance.setRpmActual();
                _instance.move();
                _instance.startTimerEngine();
            }
            else if (_instance._degree_actual > (_instance._degree_target + _instance._degree_per_step / 2)) {
                _instance.setDirection(ENGINE_DIRECTION::cw);
                _instance.setDegreeActual();
                _instance.setRpmActual();
                _instance.move();
                _instance.startTimerEngine();
            } 
            else {
                _instance.setDirection(ENGINE_DIRECTION::undefined);
                _instance.setDegreeActual();
                _instance.setRpmActual();
                _instance.setPosition(false);
                _instance.stopTimerEngine();
            }
        }   
    }
}


/*      PUBLIC METHODS      */

// SETTER

void       STEPPER_ENGINE::setDegreeTargetMax(float degree_target_max) {
    _degree_target_max = static_cast<int32_t>(degree_target_max * 1000);
}

void        STEPPER_ENGINE::setDegreeTarget(float degree_target) {
    setPosition(true);

    int32_t degree_target_local = static_cast<int32_t>(degree_target * 1000);

    if (degree_target_local > _degree_target_max)           _degree_target = _degree_target_max;
    else if (degree_target_local < - _degree_target_max)    _degree_target = - _degree_target_max;
    else                                                    _degree_target = degree_target_local;
    
    if (getState() == false) {
        startTimerEngine();
    }
}

void        STEPPER_ENGINE::setRpmMin(uint16_t rpm_min) {
    if (_setPosition == false) return;

    stopTimerEngine();
    _rpm_min = rpm_min;

    setPrescaler(1.0 * _rpm_min);
    startTimerEngine();
}

void        STEPPER_ENGINE::setRpmMax(uint16_t rpm_max) {
    _rpm_max = rpm_max;
}

void        STEPPER_ENGINE::setStepsPerRevolution(uint16_t steps_per_revolution) {
    switch (_step_mode) {
        case ENGINE_STEP_MODE::fullstep : _steps_per_revolution = steps_per_revolution;     break;
        case ENGINE_STEP_MODE::halfstep : _steps_per_revolution = 2 * steps_per_revolution; break;
        default : _steps_per_revolution = steps_per_revolution;
    }

    setDegreePerStep(_steps_per_revolution);
}

// GETTER
bool        STEPPER_ENGINE::getState(void) const {
    return TCCR1B & (_BV(CS12) | _BV(CS11) | _BV(CS10));
}

float      STEPPER_ENGINE::getDegreeActual(void) const {
    return static_cast<float>(_degree_actual / 1000.0);   
}

float      STEPPER_ENGINE::getDegreeTarget(void) const {
    return static_cast<float>(_degree_target / 1000.0);
}

float      STEPPER_ENGINE::getDegreeTargetMax(void) const {
    return static_cast<float>(_degree_target_max / 1000.0);
}

float      STEPPER_ENGINE::getFreq(void) const {
    return _freq;
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

    SET_GPIO_DIRECTION_END_POSITON_PUSHBUTTON();
    SET_GPIO_DIRECTION_ENGINE();
    H_L1CW_L2CCW();

    // turn on CTC mode - Clear Timer on Compare 
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // set entire TCCR1B register to 0

    TCCR1B = _BV(WGM12);   // enable Clear Timer on Compare immedately 
    TIMSK1 = _BV(OCIE1A);  // enable timer 1 compare interrupt for OCR1A

    float    degree_max_local = getDegreeTargetMax();
    uint16_t rpm_max_local    = getRpmMax();
    uint32_t counter_steps    = 0;

    setDegreeTargetMax(360.0);
    setRpmMax(50);

    setDegreeActual(true);
    startTimerEngine();

    while (CHECK_END_POSITION_LEFT()) {
        setPosition(true);
        setDegreeTarget(static_cast<float>((_degree_actual + _degree_per_step) / 1000.0));
        while (_degree_actual != _degree_target) {}
    }

    while (getState()) {}

    while (CHECK_END_POSITION_RIGHT()) {
        counter_steps++;
        setPosition(true);
        setDegreeTarget(static_cast<float>((_degree_actual - _degree_per_step) / 1000.0));
        while (_degree_actual != _degree_target) {}
    }

    while (getState()) {}

    for (uint16_t i = 0; i < counter_steps / 2; i++) {
        setPosition(true);
        setDegreeTarget(static_cast<float>((_degree_actual + _degree_per_step) / 1000.0));
        while (_degree_actual != _degree_target) {}
    }

    while (getState()) {}


    setDegreeTargetMax(degree_max_local);
    setRpmMax(rpm_max_local);
    setDegreeActual(true);
    Serial.println("Finished initialization!");
}

void        STEPPER_ENGINE::stop(void) {
    setDegreeTarget(0.0);

    while (getState()) {};

    Serial.println("Finish");
}

void STEPPER_ENGINE::setOCR(uint16_t ocr) {    
    _ocr = ocr; 
    Serial.println(String(_freq, 4) + "\t" + String(_timer_prescaler) + "\t" + String(_ocr) + "\t" + String(_degree_actual/1000.0, 3));
}

void STEPPER_ENGINE::setPosition(bool setPosition) {
    _setPosition = setPosition;
}

void STEPPER_ENGINE::setFreq(float degree) {
    if (_setPosition) return;
    static  unsigned    long  t_old = 0;                // old time value in ms
                        float t_now = micros() / 1000;  // actual timestamp in ms
                        float t_1   = 100.0;            // T1 in ms

    float filter_factor = t_1 / (t_now - t_old );
    t_old = t_now;

    _freq = ((_freq * filter_factor) + degree) / (filter_factor + 1.0);  // compute the angular velocity -> P-T1 element

    if (0.1 < _freq) {
        setDirection(ENGINE_DIRECTION::ccw);
        setPrescaler(_freq);
        if (getState() == false) {
            startTimerEngine();
        }
    }
    else if (_freq < -0.1) {
        setDirection(ENGINE_DIRECTION::cw);
        setPrescaler(-1.0 * _freq);
        if (getState() == false) {
            startTimerEngine();
        }
    }
    else {
        _freq = 0.0;
        STOP_ENGINE();
        stopTimerEngine();
    }
}