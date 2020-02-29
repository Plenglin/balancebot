#pragma once


template <int n>
/**
 * A filter that can be used as a moving average or rolling sum.
 */
class SMA {
private:
    float buf[n] = {0};
    int index = 0;
    float sum = 0;
public:
    SMA() {

    }
    float push(float x) {
        float old = buf[index];
        buf[index] = x;
        sum += x - old;
        if (index == n) {
            index = 0;
        }
        return old;
    }

    float getSum() {
        return sum;
    }

    float getAvg() {
        return sum / n;
    }
};

class DCBlocker {
private:
    float xp = 0;
    float yp = 0;
    float k;
public:
    DCBlocker(float k);
    float push(float x);
};
