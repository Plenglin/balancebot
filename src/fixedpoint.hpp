#pragma once
#include <Arduino.h>

template <int offset>
class FixedPoint {
private:
    long value;
public:
    FixedPoint() : value(0) {
    }

    FixedPoint(long value) : value(value) {
    }

    FixedPoint(long upper, long lower) : value((upper << offset) | lower) {
    }

    FixedPoint<offset> operator +(FixedPoint<offset> other) {
        return FixedPoint<offset>(value + other.value);
    }
    
    FixedPoint<offset> operator -(FixedPoint<offset> other) {
        return FixedPoint<offset>(value - other.value);
    }
    
    FixedPoint<offset> operator *(FixedPoint<offset> other) {
        return FixedPoint<offset>((value * other.value) >> offset);
    }
    
    FixedPoint<offset> operator /(FixedPoint<offset> other) {
        return FixedPoint<offset>(((value << offset) / other.value));
    }
    
    FixedPoint<offset> operator +(int other) {
        return FixedPoint<offset>(value + (other << offset));
    }
    
    FixedPoint<offset> operator -(int other) {
        return FixedPoint<offset>(value - (other << offset));
    }
    
    FixedPoint<offset> operator *(int other) {
        return FixedPoint<offset>(value * other);
    }
    
    FixedPoint<offset> operator /(int other) {
        return FixedPoint<offset>(value / other);
    }
    
    void operator +=(FixedPoint<offset> other) {
        value += other.value;
    }
    
    void operator -=(FixedPoint<offset> other) {
        value -= other.value;
    }
    
    void operator *=(FixedPoint<offset> other) {
        value = (value * other.value) >> offset;
    }
    
    void operator /=(FixedPoint<offset> other) {
        value = ((value << offset) / other.value);
    }
    
    void operator +=(int other) {
        value += other << offset;
    }
    
    void operator -=(int other) {
        value -= other;
    }
    
    void operator *=(int other) {
        value *= other;
    }
    
    void operator /=(int other) {
        value /= other;
    }

    bool operator <(FixedPoint<offset> other) {
        return value < other.value;
    }

    bool operator >(FixedPoint<offset> other) {
        return value > other.value;
    }

    bool operator <(int other) {
        return value < (other << offset);
    }

    bool operator >(int other) {
        return value > (other << offset);
    }

    int getUpper() {
        return value >> offset;
    }
    
    int getLower() {
        return value & ~((-1) << offset);
    }

    long getValue() {
        return value;
    }

    String toString() {
        if (*this < 0) {
            FixedPoint<offset> v = -(*this);
            return "-" + String(v.getUpper()) + "," + String(v.getLower());
        }
        return String(getUpper()) + "," + String(getLower());
    }
    
    FixedPoint<offset> operator -() {
        return FixedPoint<offset>(-value);
    }
};

using fixed = FixedPoint<8>;

/**
 * Approximates (emphasis: APPROXIMATES) an arctangent function. Error is typically <4%.
 */
fixed atan(fixed x);
