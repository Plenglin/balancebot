#include "stepstick.hpp"


StepStick::StepStick(int en, int step, int dir, int ppr, bool reverse, long maxAccel, unsigned int deadzone) :
    pin_en(en), pin_step(step), pin_dir(dir), ppr(ppr), reverse(reverse), maxAccel(maxAccel), deadzone(deadzone) {

    pinMode(en, OUTPUT);
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
    setEnabled(false);
}

void StepStick::setEnabled(bool state) {
    enabled = state;
    digitalWrite(pin_en, !state);
}

void StepStick::setVelocity(long velocity) {
    this->velocity = velocity;
    unsigned long speed = abs(velocity);
    if (velocity == 0 || speed <= deadzone) {
        direction = 0;
        return;
    }
    if (velocity < 0) {
        direction = -1;
    } else if (velocity > 0) {
        direction = 1;
    }
    period = 1000000 / (speed * ppr);
    nextStep = lastStep + period;
}

void StepStick::setTargetVelocity(long target) {
    setVelocity(target);
}

void StepStick::step(int dir) {
    nextStep = micros() + period;
    steps += dir;
    digitalWrite(pin_dir, ((dir < 0) ^ reverse) != 0);
    digitalWrite(pin_step, true);
    delayMicroseconds(STEPSTICK_MIN_PULSE);
    digitalWrite(pin_step, false);
}

void StepStick::update() {
    if (updateCount >= nextStep) {
        lastStep = updateCount;
        nextStep = updateCount + period;
        step(direction);
    }
    updateCount++;
}

long StepStick::getStepCount() {
    return steps;
}

unsigned long StepStick::getUpdateCount() {
    return updateCount;
}

