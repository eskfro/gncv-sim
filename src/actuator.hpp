#pragma once
// ============================================================================
// Actuator Symbols (Fossen)
// ============================================================================
//
//   delta   : rudder angle                         [rad]
//   n       : propeller shaft speed                [rev/s]
//   T       : propeller thrust                     [N]
//   Q       : propeller torque                     [Nm]
//   J       : advance ratio, J = u_a / (n*D)       [-]
//   K_T(J)  : thrust coefficient                   [-]
//   K_Q(J)  : torque coefficient                   [-]
//   D       : propeller diameter                   [m]
//   u_a     : advance speed (inflow at propeller)  [m/s]
//   rho     : water density                        [kg/m^3]
//
//   T = rho * D^4 * K_T(J) * n*|n|
//   Q = rho * D^5 * K_Q(J) * n*|n|
//
//   tau     : generalized force/moment from an actuator, mapped to
//             vessel reference point via lever arm (moment = r x F)
// ============================================================================
#include "common.hpp"
#include <armadillo>
#include <stdlib.h>
#include <cstdint>
#include <string_view>

namespace actuators {

class SimulatedMainPropulsion {
public:
    void Step(double dt);

    void SetRpmCommand(double rpm_command);

    const double& Rpm() const { return rpm_; }
    const double& RpmRate() const { return rpm_rate_; }
    const double& RpmCommand() const { return rpm_command_; }
    const double& MaxRpm() const { return max_rpm_; }
    const double& MinRpm() const { return min_rpm_; }
    const arma::vec3& Position() const { return position_; }
    const arma::vec6& ThrusVector() const  { return thrust_vector_; }

private:
    std::string_view name_{"MyMainPropulsor"};
    arma::vec3 position_{};
    arma::vec6 thrust_vector_{};     // (1, 0, 0, 0, 0, -l_yi)

    double time_constant_{3.0};
    double max_rpm_{600};
    double min_rpm_{-600};

    // State
    double rpm_{};
    double rpm_rate_{};
    double rpm_command_{};
};

class SimulatedRudder {
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
    arma::vec6 thrust_vector_{};        //(0, 1, 0, -l_zi, 0, l_xi) 
    
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

class SimulatedTunnelThruster {
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
    arma::vec3 thrust_vector_{};        //(0, 1, 0, -l_zi, 0, l_xi) 

    double time_constant_{1.0};
    double max_rpm_{1000};
    double min_rpm_{-1000};

    // State
    double rpm_{};
    double rpm_rate_{};
    double rpm_command_{};
};

} // namespace actuators