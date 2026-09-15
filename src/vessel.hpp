#pragma once
// ============================================================================
// 6-DOF Marine Craft Dynamics (Fossen)
// ============================================================================
//
// Equations of motion:
//
//   Kinematics:   eta_dot = J(eta) * nu
//
//   Kinetics:     M * nu_dot + C(nu) * nu + D(nu) * nu + g(eta) = tau
//
// where:
//
//   eta = [x, y, z, phi, theta, psi]^T   : position/orientation (NED frame)
//   nu  = [u, v, w, p, q, r]^T           : linear/angular velocity (body frame)
//   tau = [X, Y, Z, K, M, N]^T           : generalized forces/moments (body frame)
//
//   M     = M_RB + M_A                   : total mass/inertia matrix
//   C(nu) = C_RB(nu) + C_A(nu)           : total Coriolis-centripetal matrix
//   D(nu) = D_L + D_NL(nu)               : total damping matrix
//   g(eta)                               : restoring forces/moments (gravity + buoyancy)
//   J(eta)                               : kinematic transform, body -> NED
//
// Rearranged for integration:
//
//   nu_dot  = M^-1 * (tau - C(nu)*nu - D(nu)*nu - g(eta))
//   eta_dot = J(eta) * nu
//
// Reference:
//   T.I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control",
//   Wiley, 2011.
// ===========================================================================
#include <memory>
#include <stdlib.h>
#include <string>
#include <armadillo>
#include <vector>

#include "actuator_io.hpp"
#include "common.hpp"
#include "guidance.hpp"
#include "dynamics.hpp"
#include "controller.hpp"

namespace vessel {

/*
Vessel IO
Force calculation to be fed into the dynamics
Other calculations tied to the physical vessel
*/
class Vessel {
public:
    Vessel() = default;

    void Step(double dt);

    void CalculateForces(); // tau = tau_wind + tau_wave + tau_hyd + tau_hs + tau_control
    void SetActuatorCommands();
    
    arma::vec6 Tau() { return tau_; }
    vessel::Dynamics* Dynamics() { return &dynamics_; }

private:
    std::string_view name_{"MyVessel"};

    // Angles
    double course_angle_{}; // = yaw + crab_angle           X
    double crab_angle_{}; // = atan(v/u) = sin-1(v/U)       Bc
    double attack_angle_{}; // = atan(wr/ur)                a
    double sideslip_angle_{}; // = sin-1(vr/Ur)             B

    // Dynamics
    arma::vec6 tau_{};
    vessel::Dynamics dynamics_{};

    // Modules ish
    guidance::Guidance guidance_{};
    common::Reference reference_{};
    controller::Controller controller_{};

    // Actuators
    common::ActuatorCommands actuator_commands_{};
    std::vector<actuators_io::Rudder> rudders_{};
    std::vector<actuators_io::MainPropulsion> main_propulsors_{};
    std::vector<actuators_io::TunnelThruster> tunnel_thrusters_{};
};

/*
TODO

Thrust matrix

*/
} // namespace vessel