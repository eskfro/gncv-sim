#pragma once

#include "common.hpp"
#include "sensor.hpp"
#include <armadillo>

namespace sensor_io {

/*
Gps reciever
Returns an (x, y, z) position
*/
class GnssRx {
public:
    GnssRx() = default;

    void SetAddNoise(bool add_noise) { sensor_.SetAddNoise(add_noise); }

    bool AddNoise() { return sensor_.AddNoise(); }
    arma::vec3 Position() { return sensor_.Position(); }

private:
    sensor::SimulatedGnssRx sensor_{};

};

/*
Imu class
Returns snapshot of body acceleration and angular rates
*/
class Imu {
public:
    Imu() = default;

    void SetAddNoise(bool add_noise) { sensor_.SetAddNoise(add_noise); }

    bool AddNoise() { return sensor_.AddNoise(); }
    common::ImuSnapshot Snapshot() { return sensor_.Snapshot(); }

private:
    sensor::SimulatedImu sensor_{};
};

} // namespace sensor_io