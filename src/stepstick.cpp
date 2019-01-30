#include "stepstick.hpp"


StepStick::StepStick(int en, int step, int dir, int ppr) :
    pin_en(en), pin_step(step), pin_dir(dir), ppr(ppr) {

    pinMode(en, OUTPUT);
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
    setEnabled(true);
}

void StepStick::setEnabled(bool state) {
    if (enabled != state) {
        enabled = state;
        digitalWrite(pin_en, !state);
    }
}

void StepStick::setVelocity(long velocity) {
    if (velocity < 0) {
        direction = -1;
        velocity = -velocity;
    } else if (velocity > 0) {
        direction = 1;
    } else {
        direction = 0;
        return;
    }
    period = 1000000000 / (abs(velocity) * ppr);
}

void StepStick::step(int dir) {
    setEnabled(true);
    nextStep = micros() + period;
    digitalWrite(pin_dir, dir < 0);
    digitalWrite(pin_step, true);
    delayMicroseconds(STEPSTICK_MIN_DELAY);
    digitalWrite(pin_step, false);    
}

unsigned long StepStick::getNextStep() {
    return nextStep;
}

void StepStick::update() {
    unsigned long currentTime = micros();
    if (currentTime >= nextStep) {
        nextStep = currentTime + period;
        step(direction);
    }
}
