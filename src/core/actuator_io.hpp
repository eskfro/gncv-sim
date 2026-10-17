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

#include "core/actuator.hpp"
#include "core/common.hpp"

/*
IO wrapper for the actuators
Reduces coupling 
Makes it easier to switch to real hw if i ever want to do that :)
*/

namespace actuators_io {

class MainPropulsion {
public:
    void Step(double dt, double u_r) { main_propulsor_.Step(dt, u_r); }

    void SetRpmCommand(double n_c) { main_propulsor_.SetRpmCommand(n_c); }

    const double& Rpm() const { return main_propulsor_.Rpm(); }
    const double& RpmRate() const { return main_propulsor_.RpmRate(); }
    const double& RpmCommand() const { return main_propulsor_.RpmCommand(); }
    const double& MaxRpm() const { return main_propulsor_.MaxRpm(); }
    const double& MinRpm() const { return main_propulsor_.MinRpm(); }

    double ForceToCommand(double f) const { return main_propulsor_.ForceToCommand(f); }
    const arma::vec3& Position() const { return main_propulsor_.Position(); } 
    const arma::vec3& Force() const { return main_propulsor_.Force(); } 
    const arma::vec3& ForceDirection() const { return main_propulsor_.ForceDirection(); } 

private:
    actuators::SimulatedMainPropulsion main_propulsor_{};
};

class Rudder {
public:
    // Simulated rudder dynamics
    void Step(double dt, double u_r) { rudder_.Step(dt, u_r); }

    void SetAngleCommand(double delta_c) { rudder_.SetAngleCommand(delta_c); }

    const double& Angle() const { return rudder_.Angle(); }
    const double& AngleRate() const { return rudder_.AngleRate(); }
    const double& AngleCommand() const { return rudder_.AngleCommand(); };

    double ForceToCommand(double f, double u) const { return rudder_.ForceToCommand(f, u); }
    const arma::vec3& Force() const { return rudder_.Force(); }
    const arma::vec3& Position() const { return rudder_.Position(); }
    const arma::vec3& ForceDirection() const { return rudder_.ForceDirection(); }

private:
    actuators::SimulatedRudder rudder_{};
}; 

class TunnelThruster {
public:
    void Step(double dt, double u_r) { tunnel_thruster_.Step(dt, u_r); }

    void SetRpmCommand(double n_c) { tunnel_thruster_.SetRpmCommand(n_c); }

    const double& Rpm() const { return tunnel_thruster_.Rpm(); }
    const double& RpmRate() const { return tunnel_thruster_.RpmRate(); }
    const double& RpmCommand() const { return tunnel_thruster_.RpmCommand(); }
    const double& MaxRpm() const { return tunnel_thruster_.MaxRpm(); }
    const double& MinRpm() const { return tunnel_thruster_.MinRpm(); }

    double ForceToCommand(double f, double u_r) const { return tunnel_thruster_.ForceToCommand(f, u_r); }
    const arma::vec3 Position() const { return tunnel_thruster_.Position(); }
    const arma::vec3 Force() const { return tunnel_thruster_.Force(); }
    const arma::vec3 ForceDirection() const { return tunnel_thruster_.ForceDirection(); }

private:
    actuators::SimulatedTunnelThruster tunnel_thruster_{};
};

} // namespace actuators_io