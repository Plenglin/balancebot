#include <Arduino.h>

#include "pid.hpp"

PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), lastError(0), sum(0), target(0) {

}

float PID::push(float fb, float dt) {
    float error = target - fb;
    float delta = (lastError - error) / dt;  // might be slow with division
    sum = sum + error * dt;

    float out = (kp * error) + (ki * sum) + (kd * delta);

    lastError = error;
    return out;
}

void PID::setTarget(float target) {
    this->target = target;
}