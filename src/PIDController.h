#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
    public:
        PIDController(float kp, float ki, float kd, float outputMin = -255, float outputMax = 255);

        void setGains(float kp, float ki, float kd);
        void setOutputLimits(float min, float max);
        void reset();

        float compute(float setpoint, float measuredValue, float deltaTime);

    private:
        float _kp, _ki, _kd;
        float _integral;
        float _prevError;
        float _outputMin, _outputMax;
        float _maxDeltaTime = 0.5f; // Maximum delta time in seconds to prevent integral windup
};

#endif