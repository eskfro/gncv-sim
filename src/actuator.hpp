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

    void SetRpmCommand(double n_c);

    const double& Rpm() const { return n_; }
    const double& RpmRate() const { return n_rate_; }
    const double& RpmCommand() const { return n_c_; }
    const double& MaxRpm() const { return n_max_; }
    const double& MinRpm() const { return n_min_; }

    const arma::vec3& ThrustConfig() const  { return thrust_config_; }
    const arma::vec6& Tau();

private:
    // Things
    std::string_view name_ = "MyMainPropulsor";
    arma::vec3 thrust_config_ = {1, 0, 0};

    double time_constant_{3.0};
    double n_max_{600};
    double n_min_{-600};

    // Force
    double k_mp_ = 1;
    arma::vec6 tau_{};

    // State
    double n_{};
    double n_rate_{};
    double n_c_{};
};

class SimulatedRudder {
public:
    // Simulated rudder dynamics
    void Step(double dt);

    void SetAngleCommand(double delta_c);

    const double& Angle() const { return delta_r_; }
    const double& AngleRate() const { return delta_r_rate_; }
    const double& AngleCommand() const { return delta_c_; };

    const arma::vec3& ThrustConfig() const { return thrust_config_; }
    const arma::vec6& Tau(double u);

private:
    // Things
    std::string_view name_ = "MyRudder";
    double l_x_ = -10;       
    arma::vec3 thrust_config_ = {0, 1, l_x_}; 
    
    // Dynamics
    double time_constant_{1.0};
    double delta_max{common::deg2rad(35)};
    double delta_min{common::deg2rad(-35)};
    
    // Force
    double k_r_ = 1;
    arma::vec6 tau_;

    // State
    double delta_r_{};           
    double delta_r_rate_{};
    double delta_c_{};
}; 

class SimulatedTunnelThruster {
public:
    void Step(double dt);

    void SetRpmCommand(double n_c);

    const double& Rpm() const { return n_; }
    const double& RpmRate() const { return n_rate_; }
    const double& RpmCommand() const { return n_c_; }
    const double& MaxRpm() const { return n_max_; }
    const double& MinRpm() const { return n_min_; }

    const arma::vec3& ThrustConfig() const { return thrust_config_; }
    const arma::vec6& Tau();

private:
    // Things
    std::string_view name_ = "MyTunnelThruster";
    double l_x_ = 10;
    arma::vec3 thrust_config_ = {0, 1, l_x_};     

    // Dynamics
    double time_constant_{1.0};
    double n_max_{1000};
    double n_min_{-1000};

    // Force
    double k_tt_ = 1;
    arma::vec6 tau_;

    // Rpm state
    double n_{};
    double n_rate_{};
    double n_c_{};
};

} // namespace actuators