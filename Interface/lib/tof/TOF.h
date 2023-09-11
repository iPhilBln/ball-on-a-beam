#ifndef TOF_H_
#define TOF_H_
    #include <Arduino.h>
    #include <Wire.h>
    #include <VL53L0X.h>
    #include "STEPPER_ENGINE.h"

class TOF {
    private:
        inline  static  bool        _instanceTofCreated = false;
                        uint8_t     _ball_radius;
                        uint8_t     _offset;
                        uint16_t    _distance;
                        uint8_t     _stop_variable;

        VL53L0X sensor;
        STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance();

        void setOffset(void);
        void startMeasurement(void);

        // Meyers Singleton Constructor
        TOF() {}
        ~TOF() {}

        TOF(const TOF&) = delete;
        TOF& operator = (const TOF&) = delete;

    public:
        static TOF& getInstance(uint8_t ball_radius = 20);

        /*      SETTER      */
        void     setBallRadius(uint8_t ball_radius);

        /*      GETTER      */
        uint8_t  getBallRadius(void) const;
        uint8_t  getOffset(void) const;

        /*      FUNCTIONALITY       */
        uint16_t getDistance(void);
        uint16_t getDistancePure(void);
        float   getDistanceSimulink(void);

        bool    beginTof(void);
};

#endif