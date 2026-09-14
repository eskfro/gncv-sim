#pragma once

#include "common.hpp"
#include <armadillo>

namespace sensor {

/*
Used by vessel to get x, y, z, coords
*/
class GnssRx {
public:
    GnssRx() = default;
    void Step(double dt);

    arma::vec3& Position() { return position_; }

private:
    bool add_noise_{true};
    arma::vec3 position_{};

};

/*
Returns the body fixed rates
*/
class Imu {
public:
    Imu() = default;

    common::ImuSnapshot Snapshot();

private:
    double ax_;
    double ay_;
    double az_;
    double p_;
    double q_;
    double r_;
};

/*
TODO

- Constant velocity (CV) model for tracking of noisy measurements
- Implement all the fundamental sensors ships use for navigation
- Have the ability to add noise
- Create an IO layer for the sensors to reduce coupling

*/


} // namespace sensor