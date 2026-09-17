#include "sensor.hpp"
#include "common.hpp"

namespace sensor {

common::ImuSnapshot SimulatedImu::Snapshot() {
    return {ax_, ay_, az_, p_, q_, r_};
}

void SimulatedGnssRx::SetAddNoise(bool add_noise) {
    add_noise_ = add_noise;
}

void SimulatedImu::SetAddNoise(bool add_noise) {
    add_noise_ = add_noise;
}

} // namespace sensor