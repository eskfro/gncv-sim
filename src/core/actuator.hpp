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
#include <armadillo>
#include <stdlib.h>
#include <cstdint>
#include <string_view>

#include "core/common.hpp"

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
    const arma::vec6& Tau() const;

    double ForceToCommand(double f) const;

private:
    // Things
    std::string_view name_ = "MyMainPropulsor";
    arma::vec3 thrust_config_ = {1, 0, 0};

    double time_constant_{2.0};
    double n_max_{220};
    double n_min_{-220};

    // Force
    double k_mp_ = 16.0;
    arma::vec6 tau_{};

    // State
    double n_{};
    double n_rate_{};
    double n_c_{};
};

class SimulatedRudder {
public:
    // Simulated rudder dynamics
    void Step(double dt, double u_r);

    void SetAngleCommand(double delta_c);

    void UpdateTau(double u);

    const double& Angle() const { return delta_r_; }
    const double& AngleRate() const { return delta_r_rate_; }
    const double& AngleCommand() const { return delta_c_; };

    double ForceToCommand(double f, double u) const;
    const arma::vec3& Force() const { return f_vec_; }
    const arma::vec3& Position() const { return r_; } 

private:
    // Things
    std::string_view name_ = "MyRudder";
    arma::vec3 r_ = {-33, 0, -5};
    arma::vec3 f_vec_{};
    
    // Dynamics
    double time_constant_{2.0};
    double delta_max_{common::deg2rad(35)};
    double delta_min_{common::deg2rad(-35)};
    
    // Force
    double k_drag_ = 2e3;
    double k_lift_ = 7.7e3;

    // State
    double delta_r_{};           
    double delta_r_rate_{};
    double delta_c_{};
}; 

class SimulatedTunnelThruster {
public:
    void Step(double dt, double u_r);
    void SetRpmCommand(double n_c);

    const double& Rpm() const { return n_; }
    const double& RpmRate() const { return n_rate_; }
    const double& RpmCommand() const { return n_c_; }
    const double& MaxRpm() const { return n_max_; }
    const double& MinRpm() const { return n_min_; }

    double ForceToCommand(double f, double u_r) const;
    const arma::vec3& Position() const { return r_; }
    const arma::vec3& Force() const { return f_vec_; }
    const arma::vec3& ForceDirection() const { return f_dir_; }

private:
    // Things
    std::string_view name_ = "MyTunnelThruster";
    arma::vec3 r_{30, 0, 6};
    arma::vec3 f_dir_ = {0, 1, 0};
    arma::vec3 f_vec_{};
    
    // Dynamics
    double time_constant_{2.0};
    double n_max_{1000};
    double n_min_{-1000};
    
    // Parameters
    double k_tt_ = 0.1;     // Force coefficient
    double k_ur_ = 1;       // Force reduction coefficient for u_r

    // Rpm state
    double n_{};
    double n_rate_{};
    double n_c_{};
};

} // namespace actuators