#pragma once

#include <armadillo>
#include <filesystem>

#include "common.hpp"

namespace vessel {

struct VesselParams {
    // Physical stuff
    double length = 70;
    double breadth = 10;
    double m = 1.865e6;                     // rho * nabla
    arma::vec3 r_cg = {0, 0, 0};
    arma::vec3 r_cb = {0, 0, -1.3};                // x_cb = x_cg (no trim moment); z_cg - z_cb = GMt
    const double g = 9.81;                  // eple fra tre konstanten
    const double w = m * g;                 // weight
    const double b = w;                     // buoancy
    const double r44 = 3.5;                 // radius of gyration in roll
    const double r55 = 17.5;                // radius of gyration in pitch
    const double r66 = 17.5;                // radius of gyration in yaw

    double area_wp = 560;                   // waterplane area (A_wp)
    double nabla = 1820;                    // displacement
    double gmt = 1.3;                        // transverse metacentric height
    double gml = 100;                        // longitudinal metacentric heights
    double lcf = -5;                        // location of the cf (centre floation) relatice to co

    // linear damping: [X_u, Y_v, Z_w, K_p, M_q, N_r] (negative -> opposes motion)
    arma::vec6 d_l_coeffs = {-5.0e3, -3.0e5, -2.0e6, -5.0e6, -7.0e7, -4.0e7};
    // quadratic damping: [X_u|u|, Y_v|v|, Z_w|w|, K_p|p|, M_q|q|, N_r|r|]
    arma::vec6 d_n_coeffs = {-5.0e3, -1.4e5, -5.0e5, -1.0e7, -1.0e8, -3.0e8};

    // Initial conditions
    arma::vec6 eta_0{0, 0, 0, common::deg2rad(5), 0, 0};
    arma::vec6 nu_0{0, 0, 0, 0, 0, 0};
};

class Dynamics {
public: 
    Dynamics() = default;

    void Step(double dt);
    common::vec12 StateDot(const common::vec12& x, double t) const;

    void UpdateDynamicMatrices();
    void Init(std::filesystem::path vessel_config);     

    // Forces from the vessel fed into dynamics 
    void SetTau(const arma::vec6& tau);

    // Getters
    const arma::vec6& Eta() const { return eta_; }
    const arma::vec6& Nu() const { return nu_; }
    const arma::vec6& Tau() const { return tau_; }
    const common::vec12 State() const { return x_; } 
    double U() const { return common::U(nu_); }
    double Time() const { return t_; };

private:
    double t_{};

    // Vessel params
    VesselParams vessel_params_ = VesselParams();

    // State
    common::vec12 x_{};
    arma::vec6 eta_{};
    arma::vec6 nu_{};
    arma::vec6 tau_{};

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