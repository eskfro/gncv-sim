#pragma once

#include "common.hpp"

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

    common::Vec3 Position();

private:
    bool add_noise_{true};
    common::Vec3 position_{};

};

class Imu {
public:
private:
};

} // namespace sensor