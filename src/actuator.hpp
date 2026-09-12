#pragma once

#include "common.hpp"
#include <armadillo>
#include <stdlib.h>
#include <cstdint>
#include <string_view>

namespace actuators {

class MainPropulsion {
public:
    void Step(double dt);

    void SetRpmCommand(double rpm_command);

    const double& Rpm() const { return rpm_; }
    const double& RpmRate() const { return rpm_rate_; }
    const double& RpmCommand() const { return rpm_command_; }
    const double& MaxRpm() const { return max_rpm_; }
    const double& MinRpm() const { return min_rpm_; }
    const arma::vec3& Position() const { return position_; } 

private:
    std::string_view name_{"MyMainPropulsor"};
    arma::vec3 position_{};
    arma::vec3 force_vector_{};     // Body fixed force vector

    double time_constant_{3.0};
    double max_rpm_{600};
    double min_rpm_{-600};

    // State
    double rpm_{};
    double rpm_rate_{};
    double rpm_command_{};
};

class Rudder {
public:
    // Simulated rudder dynamics
    void Step(double dt);

    void SetAngleCommand(double angle_command);

    const double& Area() const { return area_; }
    const double& Angle() const { return angle_; }
    const double& AngleRate() const { return angle_rate_; }
    const double& AngleCommand() const { return angle_command_; };

private:
    std::string_view name_ = "MyRudder";
    arma::vec3 position_{};
    
    // Dynamics
    double time_constant_{1.0};
    double delta_max{common::deg2rad(35)};
    double delta_min{common::deg2rad(-35)};

    double area_{};

    // State
    double angle_{};            // delta_r
    double angle_rate_{};
    double angle_command_{};    // delta_c
}; 

class TunnelThruster {
public:
    void Step(double dt);

    void SetRpmCommand(double rpm_command);

    const double& Rpm() const { return rpm_; }
    const double& RpmRate() const { return rpm_rate_; }
    const double& RpmCommand() const { return rpm_command_; }
    const double& MaxRpm() const { return max_rpm_; }
    const double& MinRpm() const { return min_rpm_; }
    const arma::vec3& Position() const { return position_; } 

private:
    std::string_view name_{"MyTunnelThruster"};
    arma::vec3 position_{};
    arma::vec3 force_vector_{};     // Body fixed force vector

    double time_constant_{1.0};
    double max_rpm_{1000};
    double min_rpm_{-1000};

    // State
    double rpm_{};
    double rpm_rate_{};
    double rpm_command_{};
};

} // namespace actuators