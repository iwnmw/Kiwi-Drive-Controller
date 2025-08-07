#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float outputMin, float outputMax) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0;
    _prevError = 0;
    _outputMin = outputMin;
    _outputMax = outputMax;
}

// Function to set PID gains
void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Function to set output limits
void PIDController::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
}

// Function to reset the PID controller
void PIDController::reset() {
    _integral = 0;
    _prevError = 0;
}

// Function to compute the PID output
float PIDController::compute(float setpoint, float measuredValue, float deltaTime) {
    float error = setpoint - measuredValue;
    _integral += error * deltaTime;
    float derivative = (error - _prevError) / deltaTime;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    // Clamp output to the defined limits
    if (output > _outputMax) {
        output = _outputMax;
    } else if (output < _outputMin) {
        output = _outputMin;
    }

    _prevError = error; // Update previous error for next iteration
    return output;
}