#pragma once

#include <Arduino.h>

#define STEPSTICK_MIN_DELAY 2

/**
 * A StepStick effectively used as a DC motor.
 */
class StepStick {
    private:
        int pin_en;
        int pin_step;
        int pin_dir;
        
        int ppr;
        long steps;
        bool enabled;

        unsigned long nextStep;
        unsigned int period;
        int direction;
    public:
        StepStick(int en, int step, int dir, int ppr);
        void step(int dir);
        void setEnabled(bool state);
        /**
         * in cmillirevolutions/second
         */
        void setVelocity(long period);
        unsigned long getNextStep();
        void update();
};
