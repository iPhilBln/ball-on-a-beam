#ifndef STEPPER_ENGINE_H_INCLUDED
#define STEPPER_ENGINE_H_INCLUDED
    #include "common.h"
    
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

    class N {
    public:
        static N& instance() {
            static N _instance;
            return _instance;
        }
        ~N() {}
        void xyz();
    private:
        N() {}                      // verhindert, dass ein Objekt von außerhalb von N erzeugt wird.
                                    // protected, wenn man von der Klasse noch erben möchte
        N( const N& );              /* verhindert, dass eine weitere Instanz via Kopier-Konstruktor erstellt werden kann */
        N & operator = (const N &); //Verhindert weitere Instanz durch Kopie
    };

    class STEPPER_ENGINE {
        private:        
            enum class ENGINE_DIRECTION {
                undefined,
                cw,
                ccw
            };
            
            enum class ENGINE_STEP_MODE {
                halfstep,
                fullstep
            };

            ENGINE_DIRECTION        _direction_helper = ENGINE_DIRECTION::undefined;
            
            static      STEPPER_ENGINE* isrInstance; // statisches Mitglied als Zeiger auf die aktuelle Instanz

                        int32_t     _degree_target;
                        int32_t     _degree_target_max;
                        uint16_t    _rpm_min;
                        uint16_t    _rpm_max;
                        bool        _step_mode;             // true: halfstep | false: fullstep
                        uint16_t    _steps_per_revolution;
            
            volatile    int32_t     _degree_actual;
                        uint16_t    _timer_prescaler;
                        uint16_t    _rpm_actual;
                        uint8_t     _rpm_counter;
                        uint16_t    _degree_per_step;
            volatile    uint8_t     _step_counter;


                    void    setDegreeActual(int32_t degree_actual);
                    void    setRpmActual(void);
                    void    setDegreePerStep(uint16_t degree_per_step);
                    void    setStepMode(bool step_mode);
                    
                    void    turn_cw(void);
                    void    turn_ccw(void);        
                    void    startTimerEngine(void);
                    void    stopTimerEngine(void); 

            static  void    isrSetDegreeActual(void)  __asm__("__vector_17") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter1 Compare Match A 
        public:
            // Constructor
            STEPPER_ENGINE(bool step_mode = true);
            STEPPER_ENGINE(uint16_t steps_per_revolution, bool step_mode = true);
            STEPPER_ENGINE(uint16_t rpm_min, uint16_t rpm_max, uint16_t steps_per_revolution, bool step_mode = true);

            // SETTER and GETTER
            float       getDegreeActual(void);

            void        setDegreeTargetMax(float degree_target_max);
            float       getDegreeTargetMax(void);

            void        setDegreeTarget(float degree_target);
            float       getDegreeTarget(void);

            uint16_t    getRpmActual(void);

            void        setRpmMin(uint16_t rpm_min);
            uint16_t    getRpmMin(void);

            void        setRpmMax(uint16_t rpm_max);
            uint16_t    getRpmMax(void);

            void        setStepsPerRevolution(uint16_t steps_per_revolution);
            uint16_t    getStepsPerRevolution(void);

            float       getDegreePerStep(void);

            
            bool        getStepMode(void);

            // aditional functions
            void        begin(void);
            void        stop(void);
    };

#endif