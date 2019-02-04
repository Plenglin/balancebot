#include "fixedpoint.hpp"

class ComplementaryFilter {
    fixed alpha;
    fixed beta;
    fixed value;
    public:
        ComplementaryFilter(fixed kBias);
        void update(fixed absolute, fixed delta);
        fixed getValue();
};
