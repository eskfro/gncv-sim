#include "sensor.hpp"
#include "common.hpp"

namespace sensor {

common::ImuSnapshot Imu::Snapshot() {
    return {ax_, ay_, az_, p_, q_, r_};
}

} // namespace sensor