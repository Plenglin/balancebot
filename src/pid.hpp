#pragma once

class PID {
    int kp;
    int ki;
    int kd;

    long lastError;
    long sum;

    public:
        PID(int kp, int ki, int kd);
        long pushError(long error, int dt);
};
