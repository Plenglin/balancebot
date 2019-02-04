#include "complementaryfilter.hpp"

#include "fixedpoint.hpp"

ComplementaryFilter::ComplementaryFilter(fixed kBias) : alpha(kBias), beta(fixed(1,0) - kBias), value(0) {

}

void ComplementaryFilter::update(fixed absolute, fixed delta) {
    value = (absolute * alpha + (value + delta) * beta);
}

fixed ComplementaryFilter::getValue() {
    return value;
}
