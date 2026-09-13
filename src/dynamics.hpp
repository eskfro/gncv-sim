#pragma once

#include <armadillo>
#include <filesystem>

#include "common.hpp"

namespace vessel {

/*
Vessel dynamics
*/
class Dynamics {
public: 
    Dynamics() = default;

    void Step(double dt);

    // Init the matrices from a vessel config
    void Init(std::filesystem::path vessel_config);     

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
    arma::mat66 M_rb_{};    // M_RB     : rigid body (mass and intertia tensor)
    arma::mat66 M_a_{};     // M_A      : added mass (hydrodynamics)
    arma::mat66 M_{};       // M        : total mass

    arma::mat66 D_l_{};     // D        : linear damping
    arma::mat66 D_n_{};     // D_n(v)   : nonlinear damping
    arma::mat66 D_{};       // D(v)     : total damping, D + D_n(v)

    arma::mat66 C_rb_{};    // C_RB(v)  : coriolis rigid body
    arma::mat66 C_a_{};     // C_A(v)   : coriolis added mass centripetal
    arma::mat66 C_{};       // C(v)     : total coriolos, C_RB(v) + C_A(v)

    arma::vec6 g_{};        // g(eta)   : restoring forces
    arma::mat66 G_l_{};     // G        : linearized restoring forces

    arma::mat66 J_{};       // J(eta)   : kinematic transform

    arma::mat33 I0_{};       // I_co    : inertia dyadic about r_co = 0

};





} // namespace vessel