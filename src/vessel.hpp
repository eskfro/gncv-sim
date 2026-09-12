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
#include "actuator.hpp"
#include "common.hpp"
#include "guidance.hpp"

#include <stdlib.h>
#include <string>
#include <armadillo>
#include <vector>

namespace vessel {
/*
Dynamics of the vessel
*/
class Dynamics {
public: 
    Dynamics() = default;

    void Step(double dt);

    // Forces from the vessel fed into dynamics 
    void SetTau(arma::vec6 tau);

    const common::Eta& Eta() { return eta_; }
    const common::Nu& Nu() { return nu_; }

private:
    // Coordinates
    common::Eta eta_{};
    common::Nu nu_{};
    
    double length_{};
    double width_{};
    arma::vec3 cg_;         // centre of gravity
    arma::vec3 cb_;         // centre of boyancy

    // Dynamics
    arma::mat66 m_rb_{};    // M_RB     : rigid body (mass and intertia tensor)
    arma::mat66 m_a_{};     // M_A      : added mass (hydrodynamics)
    arma::mat66 m_{};       // M        : total mass

    arma::mat66 d_l_{};     // D        : linear damping
    arma::mat66 d_n_{};     // D_n(v)   : nonlinear damping
    arma::mat66 d_{};       // D(v)     : total damping, D + D_n(v)

    arma::mat66 c_rb_{};    // C_RB(v)  : coriolis rigid body
    arma::mat66 c_a_{};     // C_A(v)   : coriolis added mass centripetal
    arma::mat66 c_{};       // C(v)     : total coriolos, C_RB(v) + C_A(v)

    arma::vec6 g_{};        // g(eta)   : restoring forces
    arma::mat66 g_l_{};     // G        : linearized restoring forces

    arma::mat66 j_{};       // J(eta)   : kinematic transform

};

/*
Vessel IO
Force calculation to be fed into the dynamics
Other calculations tied to the physical vessel
*/
class Vessel {
public:
    Vessel() = default;

    void Step(double dt);

    void CalculateForces();             // Wind, hydrodynamics, actuators
    
    arma::vec6 Tau() { return tau_; }

private:
    std::string_view name_{"MyVessel"};

    common::Reference reference_{};
    arma::vec6 tau_{};

    Dynamics dynamics_{};
    guidance::Guidance guidance_{};

    // Actuators
    std::vector<actuators::Rudder> rudders_{};
    std::vector<actuators::MainPropulsion> main_propulsors_{};
    std::vector<actuators::TunnelThruster> tunnel_thrusters_{};
};

} // namespace vessel