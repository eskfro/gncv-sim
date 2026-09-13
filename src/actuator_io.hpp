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
#include "actuator.hpp"
#include "common.hpp"
#include <armadillo>
#include <stdlib.h>
#include <cstdint>
#include <string_view>

/*
IO wrapper for the actuators
Reduces coupling 
Makes it easier to switch to real hw if i ever want to do that :)
*/

namespace actuators_io {

class MainPropulsion {
public:
    void Step(double dt) { main_propulsor_.Step(dt); }

    void SetRpmCommand(double rpm_command) { main_propulsor_.SetRpmCommand(rpm_command); }

    const double& Rpm() const { return main_propulsor_.Rpm(); }
    const double& RpmRate() const { return main_propulsor_.RpmRate(); }
    const double& RpmCommand() const { return main_propulsor_.RpmCommand(); }
    const double& MaxRpm() const { return main_propulsor_.MaxRpm(); }
    const double& MinRpm() const { return main_propulsor_.MinRpm(); }
    const arma::vec3& Position() const { return main_propulsor_.Position(); } 

private:
    actuators::SimulatedMainPropulsion main_propulsor_{};
};

class Rudder {
public:
    // Simulated rudder dynamics
    void Step(double dt) { rudder_.Step(dt); }

    void SetAngleCommand(double angle_command) { rudder_.SetAngleCommand(angle_command); }

    const double& Area() const { return rudder_.Area(); }
    const double& Angle() const { return rudder_.Angle(); }
    const double& AngleRate() const { return rudder_.AngleRate(); }
    const double& AngleCommand() const { return rudder_.AngleCommand(); };

private:
    actuators::SimulatedRudder rudder_{};
}; 

class TunnelThruster {
public:
    void Step(double dt) { tunnel_thruster_.Step(dt); }

    void SetRpmCommand(double rpm_command) { tunnel_thruster_.SetRpmCommand(rpm_command); }

    const double& Rpm() const { return tunnel_thruster_.Rpm(); }
    const double& RpmRate() const { return tunnel_thruster_.RpmRate(); }
    const double& RpmCommand() const { return tunnel_thruster_.RpmCommand(); }
    const double& MaxRpm() const { return tunnel_thruster_.MaxRpm(); }
    const double& MinRpm() const { return tunnel_thruster_.MinRpm(); }
    const arma::vec3& Position() const { return tunnel_thruster_.Position(); } 

private:
    actuators::SimulatedTunnelThruster tunnel_thruster_{};
};

} // namespace actuators_io