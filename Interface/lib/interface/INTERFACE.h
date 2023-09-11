#ifndef CONTROLLER_H_
#define CONTROLLER_H_

    #include <Arduino.h>
    #include "STEPPER_ENGINE.h"
    #include "HC_SR04.h"
    #include "TOF.h"
    #include "simulink_interface/simulink_interface.h"

    /**
     * SLAVE ENABLE             = D08 / PH5
     * SELECT SENSOR ULTRASONIC = D11 / PB5
     * SELECT SENSOR TOF        = D12 / PB6
     * SELECT SENSOR RESISTOR   = D13 / PB7
     */

    #define _SET_GPIO_DIRECTION_SLAVE()             DDRH |= _BV(PH5); PORTH &= ~( _BV(PH5) )                                                    // set Pin PH5 as OUTPUT and LOW
    #define _SET_GPIO_DIRECTION_SENSOR_SELECT()     DDRB &= ~( _BV(PB5) | _BV(PB6) | _BV(PB7) ); PORTB &= ~( _BV(PB5) | _BV(PB6) | _BV(PB7) )   // set Pin PB5, PB6, PB7 as INPUT and deactivate internal PULL-UP resistor

    #define _SLAVE_ENABLE()     PORTH |=    _BV(PH5)
    #define _SLAVE_DISABLE()    PORTH &= ~( _BV(PH5) )

    #define _SELECT_SENSOR_ULTRASONIC() (PINB & _BV(PB5))    
    #define _SELECT_SENSOR_TOF()        (PINB & _BV(PB6)) 
    #define _SELECT_SENSOR_RESISTOR()   (PINB & _BV(PB7)) 

    enum class SENSOR {
        ULTRASONIC,
        TOF,
        RESISTOR,
        UNDEFINED
    };

    class INTERFACE {
        private:
            inline static bool  _instanceInterfaceCreated = false;

            volatile    bool    _running;
                        SENSOR  _sensor;

            void run(void);
            void selectSensor(void);
            void initSensor(void);

            void startTimer(void);
            void stopTimer(void);

            TOF*            tof;
            HC_SR04*        ultrasonic;
            STEPPER_ENGINE* stepper;     

            static  void    isrCycletime(void)  __asm__("__vector_13") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter2 Compare Match A 
            
            // Meyers Singleton Constructor
            INTERFACE() {}
            ~INTERFACE() {}
            
            INTERFACE(const INTERFACE&) = delete;
            INTERFACE& operator = (const INTERFACE&) = delete;

        public:
            static INTERFACE& getInstance(uint8_t ball_radius = 20);

            // SETTER
            void setSensor(SENSOR sensor);

            // GETTER
            SENSOR getSensor(void) const;

            // FUNCTIONS
            void begin(void);
            void stop(void);
    };

#endif
