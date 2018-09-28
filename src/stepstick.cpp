#include "stepstick.hpp"

StepStick::StepStick(int en, int step, int dir, unsigned int period, bool initialEnable) :
    pin_en(en), pin_step(step), pin_dir(dir), period(period) {

    pinMode(en, OUTPUT);
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
    setEnabled(initialEnable);
}

void StepStick::setEnabled(bool state) {
    digitalWrite(pin_en, !state);
    this->enabled = state;
}

void StepStick::setPeriod(unsigned int period) {
    this->period = period;
}

void StepStick::resetSteps() {
    resetSteps(0);
}

void StepStick::resetSteps(long steps) {
    this->steps = steps;
}

void StepStick::stepBy(long steps) {
    if (steps < 0) {
        steps = -steps;
        this->steps -= steps;
        digitalWrite(pin_dir, true);
    } else {
        this->steps += steps;
        digitalWrite(pin_dir, false);
    }
    while (steps--) {
        digitalWrite(pin_step, true);
        delay(1);
        digitalWrite(pin_step, false);
        delay(period - 1);
    }
}

void StepStick::stepTo(long steps) {
    stepBy(steps - this->steps);
}