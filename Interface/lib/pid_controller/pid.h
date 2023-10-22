#ifndef PID_H_
#define PID_H_
    #include <Arduino.h>
    // Quelle: https://github.com/pms67/PID.git

    class PIDController {

            /* Controller gains */
            float _kp;
            float _ki;
            float _kd;

            /* Derivative low-pass filter time constant */
            float _tau;

            /* Output limits */
            float _out_min;
            float _out_max;

            /* Sample time (in seconds) */
            float _dt;

            /* Controller "memory" */
            float _integrator;
            float _prevError;
            float _differentiator;
            float _prevMeasurement;

            /* Controller output */
            float _out;

        public:
            PIDController(void);
            PIDController(float kp, float ki, float kd, float dt, float out_min, float out_max);

            
            void setPID(float setpoint, float measurement);
            void setKP(float kp);
            void setKI(float ki);
            void setKD(float kd);

            float getPID(void) const;
    };
#endif