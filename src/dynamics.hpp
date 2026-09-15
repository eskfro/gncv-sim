#pragma once

#include <armadillo>
#include <filesystem>

#include "common.hpp"

namespace vessel {

struct VesselParams {
    // Physical stuff
    double length = 70;
    double breadth = 10;
    double m = 100000;
    arma::vec3 r_cg = {5, 0, 0};
    arma::vec3 r_cb = {0, 0, 7};
    const double g = 9.81;                  // eple fra tre konstanten
    const double w = m * g;                 // weight
    const double b = w;                     // buoancy
    const double r44 = 10;                  // radius of gyration in roll
    const double r55 = 10;                  // radius of gyration in pitch
    const double r66 = 10;                  // radius of gyration in yaw

    double area_wp = 1000;                  // waterplane area (A_wp)
    double nabla = 10000;                   // displacement
    double gmt = 2;                         // transverse metacentric height
    double gml = 8;                         // longitudinal metacentric heights
    double lcf = -5;                        // location of the cf (centre floation) relatice to co

    // linear damping: [X_u, Y_v, Z_w, K_p, M_q, N_r] (negative -> opposes motion)
    arma::vec6 d_l_coeffs = {-5.0e4, -1.0e5, -5.0e4, -1.0e6, -1.0e6, -5.0e6};
    // quadratic damping: [X_u|u|, Y_v|v|, Z_w|w|, K_p|p|, M_q|q|, N_r|r|]
    arma::vec6 d_n_coeffs = {-5.0e3, -1.0e4, -5.0e3, -1.0e5, -1.0e5, -5.0e5};

    // Initial conditions
    common::Eta eta_0{};
    common::Nu nu_0{};
};

class Dynamics {
public: 
    Dynamics() = default;

    void Step(double dt);

    void UpdateDynamicMatrices();
    void Init(std::filesystem::path vessel_config);     

    // Forces from the vessel fed into dynamics 
    void SetTau(arma::vec6 tau);

    const common::Eta& Eta() { return eta_; }
    const common::Nu& Nu() { return nu_; }

private:
    // Vessel params
    VesselParams vessel_params_ = VesselParams();

    // State
    common::vec12 x_;
    common::Eta eta_{};
    common::Nu nu_{};

    // Dynamics
    arma::mat66 D_n_{};     // D_n(v)   : nonlinear damping
    arma::mat66 D_{};       // D(v)     : total damping, D_l + D_n(v)
    arma::mat66 C_rb_{};    // C_RB(v)  : coriolis rigid body
    arma::mat66 C_a_{};     // C_A(v)   : coriolis added mass centripetal
    arma::mat66 C_{};       // C(v)     : total coriolos, C_RB(v) + C_A(v)
    arma::vec6 g_{};        // g(eta)   : restoring forces 
    arma::mat66 J_{};       // J(eta)   : kinematic transform

    arma::mat66 M_rb_{};    // M_RB     : rigid body (mass and intertia tensor)
    arma::mat66 M_a_{};     // M_A      : added mass (hydrodynamics)
    arma::mat66 M_{};       // M        : total mass
    arma::mat66 D_l_{};     // D_l      : linear damping
    arma::mat66 G_{};       // G        : linearized restoring forces
    arma::mat33 I_co_{};    // I_co     : inertia dyadic about r_co = 0

};

} // namespace vessel