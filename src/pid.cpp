#include <Arduino.h>

#include "pid.hpp"

PID::PID(fixed kp, fixed ki, fixed kd) : kp(kp), ki(ki), kd(kd), lastError(0), sum(0), target(0) {

}

fixed PID::pushError(fixed error, int dt) {
    fixed delta = (lastError - error) / dt;  // might be slow with division
    sum = sum + error * dt;

    fixed out = (kp * error) + (ki * sum) + (kd * delta);

    lastError = error;
    return out;
}

void PID::setTarget(fixed target) {
    this->target = target;
}