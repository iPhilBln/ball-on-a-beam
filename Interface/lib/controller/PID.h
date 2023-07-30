#ifndef PID_H_
#define PID_H_

#include <Arduino.h>
#include "STEPPER_ENGINE.h"
#include "HC_SR04.h"

#ifdef __cplusplus
extern "C" {
#endif

// Externe "C"-Deklaration der C++-Funktionen
double mapd(double x, double in_min, double in_max, double out_min, double out_max);

#ifdef __cplusplus
}
#endif


enum class SENSOR {
    ULTRASONIC,
    TOF,
    CAPAITY,
    RESISTOR,
    undefined
};

class CONTROLLER {
    private:
        inline static bool _instanceCreated = false;

        float _kp;
        float _ki;
        float _kd;

        double _integral;
        double _derivate;
        double _error;
        double _pre_error;

        volatile    bool   _running;
                    SENSOR _sensor;

        void setTimer(void);

        //HC_SR04* ultrasonicPtr = nullptr;
        //HC_SR04&        ultrasonic = HC_SR04::getInstance();
        STEPPER_ENGINE& stepper = STEPPER_ENGINE::getInstance();

        static  void    isrCycletime(void)  __asm__("__vector_13") __attribute__((__signal__, __used__, __externally_visible__)); // Timer/Counter2 Compare Match A 
        
        // Meyers Singleton Constructor
        CONTROLLER() {}
        ~CONTROLLER() {}
        
        CONTROLLER(const CONTROLLER&) = delete;
        CONTROLLER& operator = (const CONTROLLER&) = delete;

    public:
        static CONTROLLER& getInstance(SENSOR sensor, float kp, float ki, float kd, bool force = false);

        // SETTER
        void setSensor(SENSOR sensor);
        void setKp(float kp);
        void setKi(float ki);
        void setKd(float kd);

        // GETTER
        SENSOR getSensor(void);
        float getKp(void);
        float getKi(void);
        float getKd(void);

        // FUNCTIONS
        void begin(void);
        void stop(void);

        void runPcontroller(void);
        void runPIcontroller(void);
        void runPDcontroller(void); 
        void runPIDcontroller(void);

        void runStepresponseOpenLoop(float step);
        void runStepresponseClosedLoop(uint16_t setpoint);
        void testEngine(void);
};

#endif
