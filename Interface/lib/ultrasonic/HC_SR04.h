#ifndef HC_SR04_H_
#define HC_SR04_H_
    #include <Arduino.h>

    /**
    * ultrasonic sensor echo left     = D49 / L0  -> ICP4
    * ultrasonic sensor echo right    = D48 / L1  -> ICP5
    * ultrasonic sensor trigger left  = D05 / E3
    * ultrasonic sensor trigger right = D04 / G5
    */

    #define _SET_GPIO_DIRECTION_ULTRASONIC_ECHO_LEFT()     DDRL &= ~( _BV(PL0) ); PORTL &= ~( _BV(PL0) ) // set Pin PL0 as INPUT and deactivate internal Pull-Up resistor
    #define _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_LEFT()  DDRE |=    _BV(PE3);   PORTE |=    _BV(PE3)   // set Pin E3 as OUTPUT and activate internal Pull-Up resistor
//    #define _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_LEFT()  DDRG |=    _BV(PE3);   PORTE &=  ~( _BV(PE3) )  // set Pin E3 as OUTPUT and activate internal Pull-Up resistor

    #define _SET_GPIO_DIRECTION_ULTRASONIC_ECHO_RIGHT()    DDRL &= ~( _BV(PL1) ); PORTL &= ~( _BV(PL1) ) // set Pin L1 as INPUT and deactivate internal Pull-Up resistor
    #define _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER_RIGHT() DDRG |=    _BV(PG5);   PORTG |=    _BV(PG5)   // set Pin G5 as OUTPUT and activate internal Pull-Up resistor

    #define _ULTRASONIC_TRIGGER_LEFT_STOP()   (PORTE |=    _BV(PE3)  )  // set D05 HIGH
    #define _ULTRASONIC_TRIGGER_LEFT_START()  (PORTE &= ~( _BV(PE3) ))  // set D05 LOW
    #define _ULTRASONIC_TRIGGER_RIGHT_STOP()  (PORTG |=    _BV(PG5)  )  // set D04 HIGH
    #define _ULTRASONIC_TRIGGER_RIGHT_START() (PORTG &= ~( _BV(PG5) ))  // set D04 LOW

    #define _ULTRASONIC_CHECK_LEVEL_ECHO_LEFT()  (PINL & _BV(PL0) ) // check if Pin L0 is HIGH and return TRUE
    #define _ULTRASONIC_CHECK_LEVEL_ECHO_RIGHT() (PINL & _BV(PL1) ) // check if Pin L1 is HIGH and return TRUE

    class HC_SR04 {
        private:
                                double      _temperature;
                                double      _ultrasonic_speed;
                                uint8_t     _ball_radius;
                                uint8_t     _timer_prescaler;

            inline  static  volatile    uint16_t    _echo_counter_left;
            inline  static  volatile    uint16_t    _echo_counter_right;
            inline  static  volatile    bool        _echo_timeout;

                                uint8_t     _offset_left;
                                uint16_t    _distance_left;

                                uint8_t     _offset_right;
                                uint16_t    _distance_right;

                                uint16_t    _distance;

                    void    setOffsetLeft(void);
                    void    setOffsetRight(void);
            
                    void    setUltrasonicSpeed(double temperature);

                    void    startTimerEchoLeft(void);
                    void    stopTimerEchoLeft(void);
                    void    startTimerEchoRight(void);
                    void    stopTimerEchoRight(void);

            static  void    isrEchoLeftCapture(void)  __asm__("__vector_41") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter4 Capture Event
            static  void    isrEchoLeftTimeout(void)  __asm__("__vector_42") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter4 Compare Match A  
            static  void    isrEchoRightCapture(void) __asm__("__vector_46") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter5 Capture Event
            static  void    isrEchoRightTimeout(void) __asm__("__vector_47") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter5 Compare Match A 
            
        public:
            HC_SR04();
            HC_SR04(uint8_t ball_radius, uint8_t timer_prescaler = 1);
            HC_SR04(double  temperature, uint8_t timer_prescaler = 1);
            HC_SR04(uint8_t ball_radius, double  temperature, uint8_t timer_prescaler = 1);

            double  getUltrasonicSpeed(void);

            void    setTimerPrescaler(uint8_t timer_prescaler);
            uint8_t getTimerPrescaler(void);

            void    setBallRadius(uint8_t ball_radius);
            uint8_t getBallRadius(void);

            void    setTemperature(double temperature);
            double  getTemperature(void);

            uint8_t getOffsetLeft(void);
            uint8_t getOffsetRight(void);

            uint16_t getDistanceSensorLeft(void);
            uint16_t getDistanceSensorRight(void);

            uint16_t getDistance(void);

            void beginSensorLeft(void);
            void beginSensorRight(void);
    };
#endif