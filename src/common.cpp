#include "common.hpp"

#include <math.h>

namespace common {

double deg2rad(double deg) {
    return M_PI * deg / 180.0;
}

} // namespace common