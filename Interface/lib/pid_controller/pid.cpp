#include "PID.h"

PIDController::PIDController(void) :
    _integrator(0.0F), 
    _prevError(0.0F), 
    _differentiator(0.0F), 
    _prevMeasurement(0.0F), 
    _out(0.0F)
    {}

PIDController::PIDController(float kp, float ki, float kd, float dt, float out_min, float out_max) :
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _dt(dt),
    _out_min(out_min),
    _out_max(out_max),
    _integrator(0.0F), 
    _prevError(0.0F), 
    _differentiator(0.0F), 
    _prevMeasurement(0.0F), 
    _out(0.0F)
    {}

/*      PUBLIC METHODS      */

// SETTER

void PIDController::setPID(float setpoint, float measurement) {
    /* Error signal */
    float error = setpoint - measurement;

    /* Proportional */
    float proportional = _kp * error;

    /* Integral */
    _integrator = _integrator + 0.5F * _ki * _dt * (error + _prevError);

    /* Anti-wind-up via integrator clamping */
    if (_integrator > _out_max) {
        _integrator = _out_max;
    } else if (_integrator < _out_min) {
        _integrator = _out_min;
    }

    /* Derivative (band-limited differentiator) */
    _differentiator = -(2.0F * _kd * (measurement - _prevMeasurement) + (2.0F * _tau - _dt) * _differentiator) / (2.0F * _tau + _dt);

    /* Compute output and apply limits */
    _out = proportional + _integrator + _differentiator;

    if (_out > _out_max) {
        _out = _out_max;
    } else if (_out < _out_min) {
        _out = _out_min;
    }

    /* Store error and measurement for later use */
    _prevError       = error;
    _prevMeasurement = measurement;
}

void PIDController::setKP(float kp) {
    _kp = kp;
}

void PIDController::setKI(float ki) {
    _ki = ki;
}

void PIDController::setKD(float kd) {
    _kd = kd;
    if (_kp != 0.0F) {
        _tau = _kd / _kp * 0.05F;
    }
    else {
        _tau = 0.001F;
    }
}

// GETTER

float PIDController::getPID() const {
    /* Return controller output */
    return _out;
}