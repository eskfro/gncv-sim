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

#include "core/config.hpp"
#include "core/common.hpp"

namespace actuators {

class SimulatedMainPropulsion {
public:
    void Step(double dt, double u_r);
    void SetRpmCommand(double n_c);

    const double& Rpm() const { return n_; }
    const double& RpmRate() const { return n_rate_; }
    const double& RpmCommand() const { return n_c_; }
    const double& MaxRpm() const { return p_.n_max; }
    const double& MinRpm() const { return p_.n_min; }

    double ForceToCommand(double f) const;
    const arma::vec3& Force() const { return f_vec_; };
    const arma::vec3& Position() const { return p_.r; };
    const arma::vec3& ForceDirection() const { return p_.f_dir; }

private:
    config::MainPropulsorParams p_{};
    
    // State
    arma::vec3 f_vec_{};
    double n_{};
    double n_rate_{};
    double n_c_{};
};

class SimulatedRudder {
public:
    // Simulated rudder dynamics
    void Step(double dt, double u_r);

    void SetAngleCommand(double delta_c);

    const double& Angle() const { return delta_r_; }
    const double& MaxAngle() const { return p_.delta_max; }
    const double& AngleRate() const { return delta_r_rate_; }
    const double& AngleCommand() const { return delta_c_; };

    double ForceToCommand(double f, double u) const;
    const arma::vec3& Force() const { return f_vec_; }
    const arma::vec3& Position() const { return p_.r; } 
    const arma::vec3& ForceDirection() const { return p_.f_dir; }

private:
    config::RudderParams p_{};
    
    // State
    arma::vec3 f_vec_{};
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
    const double& MaxRpm() const { return p_.n_max; }
    const double& MinRpm() const { return p_.n_min; }

    double ForceToCommand(double f, double u_r) const;
    const arma::vec3& Position() const { return p_.r; }
    const arma::vec3& Force() const { return f_vec_; }
    const arma::vec3& ForceDirection() const { return p_.f_dir; }

private:
    config::TunnelThrusterParams p_{};
    
    // State
    arma::vec3 f_vec_{};
    double n_{};
    double n_rate_{};
    double n_c_{};
};

} // namespace actuators