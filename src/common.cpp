#include "common.hpp"

#include <math.h>

namespace common {

double deg2rad(double deg) {
    return M_PI * deg / 180.0;
}

bool inrange(double value, double range_min, double range_max) {
    /*
    Check if a value is inside a range
    Helper to make code more readable
    */
    return range_min < value && value < range_max;
}

} // namespace common