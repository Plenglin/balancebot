#pragma once

#include <Arduino.h>

class StepStick {
    int pin_en;
    int pin_step;
    int pin_dir;

    long steps;
    unsigned int period;
    bool enabled;
    public:
        StepStick(int en, int step, int dir, unsigned int period, bool initialEnable);
        void stepTo(long steps);
        void stepBy(long steps);
        void setEnabled(bool state);
        void setPeriod(unsigned int period);
        long getSteps();
        void resetSteps(long steps);
        void resetSteps();
};
