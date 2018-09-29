#pragma once

class PID {
    private:
        int kp;
        int ki;
        int kd;

        long lastError;
        long sum;
        long target;
    public:
        PID(int kp, int ki, int kd);
        long pushError(long error, int dt);
        void setTarget(long target);
};
