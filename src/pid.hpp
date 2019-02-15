#pragma once

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
        float push(float error, float dt);
        void setTarget(float target);
};
