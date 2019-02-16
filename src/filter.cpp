#include "filter.hpp"

DCBlocker::DCBlocker(float k) : k(k) {

}

float DCBlocker::push(float x) {
    float y = x - xp + k * yp;
    xp = x;
    yp = y;
    return y;
}
