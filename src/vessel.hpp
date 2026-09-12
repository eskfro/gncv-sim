#pragma once

#include "actuator.hpp"
#include "common.hpp"

#include <stdlib.h>
#include <string>
#include <armadillo>
#include <vector>

/*
Dynamics of the vessel
*/
class Dynamics {
public: 
    Dynamics() = default;

    void Step(double dt);

    // Forces from the vessel fed into dynamics 
    void SetExternalForces(double external_forces);

    const common::Eta& Eta() { return eta_; }
    const common::Nu& Nu() { return nu_; }

private:
    // Coordinates
    common::Eta eta_{};
    common::Nu nu_{};
    
    // Physical stuff
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

private:
    std::string_view name_{"MyVessel"};

    common::Eta eta_c_{};
    common::Nu nu_c_{};

    Dynamics dynamics_{};

    // Actuators
    std::vector<actuators::Rudder> rudders_{};
    std::vector<actuators::MainPropulsion> main_propulsors_{};
    std::vector<actuators::TunnelThruster> tunnel_thrusters_{};
};