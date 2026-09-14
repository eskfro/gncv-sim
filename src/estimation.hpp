#pragma once

#include <armadillo>

#include "common.hpp"

namespace estimation {

/*
Estimate init state from two first measurements on a cv model
*/
void get_init_cv_state(double meas0, double meas1, int ekf_params);

} // namespace estimation