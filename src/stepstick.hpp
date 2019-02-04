#pragma once

#include <Arduino.h>

#define STEPSTICK_MIN_DELAY 10

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
        bool reverse;

        unsigned long nextStep;
        unsigned long lastUpdate;
        long maxAccel;
        unsigned int period;
        long targetVelocity = 0;
        long velocity = 0;
        int direction = 0;
        void setVelocity(long velocity);
    public:
        StepStick(int en, int step, int dir, int ppr, bool reverse, long maxAccel);
        void step(int dir);
        void setEnabled(bool state);
        /**
         * in millirevolutions/second
         */
        void setTargetVelocity(long velocity);
        unsigned long getNextStep();
        void update();
};
