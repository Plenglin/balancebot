#include "pid.hpp"

PID::PID(int kp, int ki, int kd) : kp(kp), ki(ki), kd(kd), lastError(0), sum(0) {

}

long PID::pushError(long error, int dt) {
    long delta = (lastError - error) / dt;  // might be slow with division
    sum += error * dt;

    long out = kp * error + ki * sum + kd * delta;

    lastError = error;
    return out;
}