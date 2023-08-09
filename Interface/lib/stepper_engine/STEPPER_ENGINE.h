#ifndef STEPPER_ENGINE_H_INCLUDED
#define STEPPER_ENGINE_H_INCLUDED
    #include <Arduino.h>
    #define pi 3.1415926535f
    /**
    * end position left  = D28 / A6
    * end position right = D29 / A7
    */
    #define SET_GPIO_DIRECTION_END_POSITON_PUSHBUTTON() DDRA &= ~( _BV(PA6) | _BV(PA7) ); PORTA |= _BV(PA6) | _BV(PA7)  // set D06 and D07 as INPUT and activate internal Pull-Up resistor
    #define CHECK_END_POSITION_LEFT()   (PINA & _BV(PA6))  // check if D06 is HIGH == TRUE -> no end position detected | LOW == FALSE -> end position is detected
    #define CHECK_END_POSITION_RIGHT()  (PINA & _BV(PA7))  // check if D07 is HIGH == TRUE -> no end position detected | LOW == FALSE -> end position is detected

    /**
    * IN 1 = D24 / A2
    * IN 2 = D25 / A3
    * IN 3 = D26 / A4
    * IN 4 = D27 / A5
    */

    // define GPIO pins for engine as output
    #define SET_GPIO_DIRECTION_ENGINE()  DDRA |= _BV(PA2) | _BV(PA3) | _BV(PA4) | _BV(PA5)

    // define GPIO register for stop the engine
    #define STOP_ENGINE()     PORTA &= ~( _BV(PA2) | _BV(PA3) | _BV(PA4) | _BV(PA5) );

    // define GPIO register for left and right direction : FULLSTEP
    #define F_L1CW_L2CCW()    PORTA &= ~( _BV(PA3) | _BV(PA4) ); PORTA |= _BV(PA2) | _BV(PA5) 
    #define F_L1CW_L2CW()     PORTA &= ~( _BV(PA3) | _BV(PA5) ); PORTA |= _BV(PA2) | _BV(PA4) 
    #define F_L1CCW_L2CW()    PORTA &= ~( _BV(PA2) | _BV(PA5) ); PORTA |= _BV(PA3) | _BV(PA4) 
    #define F_L1CCW_L2CCW()   PORTA &= ~( _BV(PA2) | _BV(PA4) ); PORTA |= _BV(PA3) | _BV(PA5) 

    // define GPIO register for left and right direction : HALFSTEP 
    #define H_L1CW_L2CCW()    PORTA &= ~( _BV(PA3) | _BV(PA4) );            PORTA |= _BV(PA2) | _BV(PA5)
    #define H_L1CW_L2OFF()    PORTA &= ~( _BV(PA3) | _BV(PA4) | _BV(PA5) ); PORTA |= _BV(PA2)
    #define H_L1CW_L2CW()     PORTA &= ~( _BV(PA3) | _BV(PA5) );            PORTA |= _BV(PA2) | _BV(PA4)
    #define H_L1OFF_L2CW()    PORTA &= ~( _BV(PA2) | _BV(PA3) | _BV(PA5) ); PORTA |= _BV(PA4)
    #define H_L1CCW_L2CW()    PORTA &= ~( _BV(PA2) | _BV(PA5) );            PORTA |= _BV(PA3) | _BV(PA4)
    #define H_L1CCW_L2OFF()   PORTA &= ~( _BV(PA2) | _BV(PA4) | _BV(PA5) ); PORTA |= _BV(PA3)
    #define H_L1CCW_L2CCW()   PORTA &= ~( _BV(PA2) | _BV(PA4) );            PORTA |= _BV(PA3) | _BV(PA5)
    #define H_L1OFF_L2CCW()   PORTA &= ~( _BV(PA2) | _BV(PA3) | _BV(PA4) ); PORTA |= _BV(PA5)


    enum class ENGINE_DIRECTION {
        undefined,
        cw,
        ccw
    };

    enum class ENGINE_STEP_MODE {
        halfstep,
        fullstep
    };
    class STEPPER_ENGINE {
        private:               
            inline  static  bool  _instanceCreated = false;

            volatile    ENGINE_DIRECTION _direction; 
                        int32_t          _degree_target;
                        int32_t          _degree_target_max;
                        uint16_t         _rpm_min;
                        uint16_t         _rpm_max;
                        ENGINE_STEP_MODE _step_mode;             
                        uint16_t         _steps_per_revolution;
            
            volatile    int32_t          _degree_actual;
                        uint16_t         _timer_prescaler;
            volatile    uint16_t         _rpm_actual;
            volatile    uint8_t          _rpm_counter;
                        uint16_t         _degree_per_step;
            volatile    uint8_t          _step_counter;

                        bool             _setPosition;
                        uint16_t         _ocr;
                        float            _freq;

                        float            _kp;
                        float            _kd;
                        float            _t1;
                        float            _dt;
                        float            _alpha;

                    //void    setDegreeActual(bool reset = false);
                    void    setRpmActual(void);
                    void    setDegreePerStep(uint16_t steps_per_revolution);
                    void    setStepMode(ENGINE_STEP_MODE step_mode);
                    void    setDirection(ENGINE_DIRECTION direction);
                    void    setPrescaler(float freq = 1.0);
                    void    setOCR(uint16_t ocr);
                    void    setPosition(bool setPosition);
                    void    setControllerPosition(void);
                    
                    void    move(void);       
                    void    startTimerEngine(void);
                    void    stopTimerEngine(void); 
                    //void    computeFreq(void);

            static  void    isrMoveEngine(void)  __asm__("__vector_17") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter1 Compare Match A 
            
            // Meyers Singleton Constructor
            STEPPER_ENGINE() {}
            ~STEPPER_ENGINE() {}
            
            STEPPER_ENGINE(const STEPPER_ENGINE&) = delete;
            STEPPER_ENGINE& operator = (const STEPPER_ENGINE&) = delete;
        public:
            static STEPPER_ENGINE& getInstance(ENGINE_STEP_MODE step_mode = ENGINE_STEP_MODE::halfstep, uint16_t rpm_min = 3, uint16_t rpm_max = 120, uint16_t steps_per_revolution = 200);

            // SETTER
            void    setDegreeActual(bool reset = false); //testen

            void    setDegreeTargetMax(float degree_target_max);
            void    setDegreeTarget(float degree_target);
            void    setRpmMin(uint16_t rpm_min);
            void    setRpmMax(uint16_t rpm_max);
            void    setStepsPerRevolution(uint16_t steps_per_revolution);
            void    setKP(float p_part);
            void    setKD(float d_part);
            void    setT1(float t1);
            void    setDT(void);
            void    setAlpha(float degree);
            
            // GETTER
            bool                getState(void) const;
            float               getDegreeActual(void) const;
            float               getDegreeActualSimulink(void) const;
            float               getDegreeTarget(void) const;
            float               getDegreeTargetMax(void) const;
            float               getFreq(void) const;
            float               getFreqSimulink(void) const;
            float               getKP(void) const;
            float               getKD(void) const;
            float               getT1(void) const;
            float               getAlpha(void) const;
            uint16_t            getRpmActual(void) const;
            uint16_t            getRpmMin(void) const;
            uint16_t            getRpmMax(void) const;
            uint16_t            getStepsPerRevolution(void) const;
            float               getDegreePerStep(void) const;
            ENGINE_STEP_MODE    getStepMode(void) const;

            // FUNCTIONS
            void        begin(void);
            void        stop(void);
    };
#endif