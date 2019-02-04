#pragma once
#include <fixedpoint.hpp>

class PID {
    private:
        fixed kp;
        fixed ki;
        fixed kd;

        fixed lastError;
        fixed sum;
        fixed target;
    public:
        PID(fixed kp, fixed ki, fixed kd);
        fixed pushError(fixed error, fixed dt);
        void setTarget(fixed target);
};
