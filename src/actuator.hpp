#pragma once

#include "common.hpp"
#include <stdlib.h>
#include <cstdint>
#include <string_view>

namespace actuators {

class Rudder {
public:
    // Simulated rudder dynamics
    void Step(double dt);

    void SetAngleCommand();

    double Angle();
    double AngleCommand();

private:
    std::string_view name_ = "MyRudder";
    
    // Dynamics
    double time_constant_{1.0};
    double delta_max{common::deg2rad(35)};
    double delta_min{common::deg2rad(-35)};

    // The rudder probably will need some constants?
    // ... we will see
    double area_{};

    // State
    double angle_{};            // delta_r
    double angle_rate_{};
    double angle_command_{};    // delta_c

}; 

} // namespace actuators