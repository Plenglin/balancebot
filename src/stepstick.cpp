#include "stepstick.hpp"


StepStick::StepStick(int en, int step, int dir, int ppr, bool reverse, long maxAccel) :
    pin_en(en), pin_step(step), pin_dir(dir), ppr(ppr), reverse(reverse), maxAccel(maxAccel), lastUpdate(micros()) {

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
    if (velocity < 0) {
        direction = -1;
        velocity = -velocity;
    } else if (velocity > 0) {
        direction = 1;
    } else {
        direction = 0;
        return;
    }
    period = 1000000000 / (velocity * ppr);
}

void StepStick::setTargetVelocity(long target) {
    this->targetVelocity = target;
}

void StepStick::step(int dir) {
    //setEnabled(true);
    nextStep = micros() + period;
    digitalWrite(pin_dir, (dir < 0) ^ reverse);
    digitalWrite(pin_step, true);
    delayMicroseconds(STEPSTICK_MIN_DELAY);
    digitalWrite(pin_step, false);    
}

unsigned long StepStick::getNextStep() {
    return nextStep;
}

void StepStick::update() {
    unsigned long currentTime = micros();
    unsigned long dt = currentTime - lastUpdate;

    long vError = targetVelocity - velocity;
    if ((unsigned long) (abs(vError) * dt) < (unsigned long) (maxAccel * 1000L)) {
        setVelocity(targetVelocity);
    } else if (vError > 0) {
        setVelocity(velocity + maxAccel);
    } else if (vError < 0) {
        setVelocity(velocity - maxAccel);
    }

    if (direction != 0 && currentTime >= nextStep) {
        nextStep = currentTime + period;
        step(direction);
    }

    lastUpdate = currentTime;
}
