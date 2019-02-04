#pragma once
#include <fixedpoint.hpp>

class PID {
    private:
        float kp;
        float ki;
        float kd;

        float lastError;
        float sum;
        float target;
    public:
        PID(float kp, float ki, float kd);
        float pushError(float error, int dt);
        void setTarget(float target);
};
