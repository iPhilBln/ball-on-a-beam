#ifndef HC_SR04_H_
#define HC_SR04_H_

    #include <Arduino.h>
    #include "STEPPER_ENGINE.h"

    /**
    * ultrasonic sensor echo    = D49 / L0  -> ICP4
    * ultrasonic sensor trigger = D05 / E3
    */

    #define _SET_GPIO_DIRECTION_ULTRASONIC_ECHO()     DDRL &= ~( _BV(PL0) ); PORTL &= ~( _BV(PL0) ) // set Pin PL0 as INPUT and deactivate internal Pull-Up resistor
    #define _SET_GPIO_DIRECTION_ULTRASONIC_TRIGGER()  DDRE |=    _BV(PE3);   PORTE |=    _BV(PE3)   // set Pin E3 as OUTPUT and activate internal Pull-Up resistor

    #define _ULTRASONIC_TRIGGER_STOP()   (PORTE |=    _BV(PE3)  )  // set D05 HIGH
    #define _ULTRASONIC_TRIGGER_START()  (PORTE &= ~( _BV(PE3) ))  // set D05 LOW

    #define _ULTRASONIC_CHECK_LEVEL_ECHO()  (PINL & _BV(PL0) ) // check if Pin L0 is HIGH and return TRUE

    class HC_SR04 {
        private:
            inline  static  bool     _instanceUltrasonicCreated = false;

                        float       _temperature;
                        float       _ultrasonic_speed;
                        uint8_t     _ball_radius;

            volatile    uint16_t    _echo_counter;
            volatile    bool        _echo_timeout;
            volatile    bool        _measurement_completed;
                        uint8_t     _measurement_failed;
                        
                        uint8_t     _offset;
                        uint16_t    _distance;

                    bool    setOffset(void);

                    void    setUltrasonicSpeed(double temperature);

                    void    setupSensor(void);
                    void    startTimerEcho(void);
                    void    stopTimerEcho(void);

            static  void    isrEchoCapture(void)  __asm__("__vector_41") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter4 Capture Event
            static  void    isrEchoTimeout(void)  __asm__("__vector_45") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter4 Timer Overflow Vector             

            // Meyers Singleton Constructor
            HC_SR04() {}
            ~HC_SR04() {}
            
            HC_SR04(const HC_SR04&) = delete;
            HC_SR04& operator = (const HC_SR04&) = delete;            
        
        public:
            static HC_SR04& getInstance(uint8_t ball_radius = 20);
            
            /*      SETTER      */
            void    setBallRadius(uint8_t ball_radius);
            void    setTemperature(void);
            
            /*      GETTER      */
            uint8_t getBallRadius(void) const;
            float   getTemperature(void) const;
            float   getUltrasonicSpeed(void) const;
            uint8_t getOffset(void) const;

            /*      FUNCTIONALITY       */
            uint16_t getDistance(void);
            uint16_t getDistancePure(void);
            float    getDistanceSimulink(void);

            bool     beginUltrasonic(void);
    };
#endif