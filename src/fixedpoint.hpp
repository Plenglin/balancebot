#pragma once

template <int offset>
class FixedPoint {
private:
    long value;
public:
    FixedPoint(long value) : value(value) {
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
        return FixedPoint<offset>(((value << offset) / other.value) >> offset);
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
        value = ((value << offset) / other.value) >> offset;
    }
    
};

