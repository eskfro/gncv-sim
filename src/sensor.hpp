#pragma once

#include "common.hpp"
#include <armadillo>

namespace sensor {

/*
Gps reciver

Used by vessel to get x, y, z, coords
Add realism by adding noise

Need some way to generate these signals according to the vessel system
... need to think about best way to do that
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

class Imu {
public:
private:
};

} // namespace sensor