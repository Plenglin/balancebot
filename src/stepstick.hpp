#pragma once

#include <Arduino.h>

#define STEPSTICK_MIN_PULSE 20

/**
 * A StepStick effectively used as a DC motor.
 */
class StepStick {
    private:
        int pin_en;
        int pin_step;
        int pin_dir;
        
        int ppr;
        bool enabled;
        bool reverse;

        volatile unsigned long nextStep;
        volatile unsigned long lastStep = 0;
        volatile unsigned long updateCount;
        volatile long steps;
        
        long maxAccel;
        unsigned int period;
        long targetVelocity = 0;
        long velocity = 0;
        int direction = 0;
        unsigned int deadzone;
        void setVelocity(long velocity);
    public:
        StepStick(int en, int step, int dir, int ppr, bool reverse, long maxAccel, unsigned int deadzone = 0);
        void step(int dir);
        void setEnabled(bool state);
        /**
         * in millirevolutions/second (i.e. v = 1000 is 1 rev/sec)
         */
        void setTargetVelocity(long velocity);
        /**
         * Expected to be called at 1024Hz
         */
        void update();

        unsigned long getUpdateCount();
        long getStepCount();
};
