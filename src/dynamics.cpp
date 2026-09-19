#include "dynamics.hpp"
#include "common.hpp"
#include <armadillo>
#include <cmath>
#include <filesystem>

/*
Dynamics (Fossen)

eta = (x, y, z, phi, theta, psi)    : generalized positions
nu = (u, v, w, p, q, r)             : generalized velocity
(X, Y, Z, K, M, N)                  : external forces and moments

tau_i = [Xi, Yi, Zi, Ki, Mi, Ni] , i element in (hyd, hs, wind, wave, control)
tau_rb = tau_hyd + tau_hs + tau_wind + tau_wave + tau_control

Marine craft equations (Fossen 1991)

eta' = J_theta(eta)nu
M nu' + C(nu)nu + D(nu)nu + g(eta) + g0 = tau + tau_wind + tau_wave
*/
namespace vessel {

common::vec12 Dynamics::StateDot(const common::vec12& x, double t) const {
    const VesselParams p = vessel_params_;
    const arma::vec6 eta = x.subvec(0, 5);
    const arma::vec6 nu  = x.subvec(6, 11);

    const arma::mat66 J     = common::J(eta);
    const arma::mat66 C_rb  = common::C_rb(p.m, I_co_, p.r_cg, nu);
    const arma::mat66 D_n   = common::D_n(p.d_n_coeffs, nu);
    const arma::mat66 D_tot = D_l_ + D_n;
    const arma::vec6  g_vec = common::g(p.w, p.b, p.r_cg, p.r_cb, eta);

    common::vec12 x_dot{};
    x_dot.subvec(0, 5)  = J * nu;
    x_dot.subvec(6, 11) = arma::solve(M_, tau_ - C_rb*nu - D_tot*nu - g_vec);
    return x_dot;
}

void Dynamics::Step(double dt) {
    auto f = [this](const common::vec12& x, double t) { return StateDot(x, t); };

    x_ = common::solver_12d_rk4(f, x_, t_, dt);
    
    eta_ = x_.subvec(0, 5);
    nu_ = x_.subvec(6, 11);
    t_ += dt;

    UpdateDynamicMatrices();
}

void Dynamics::Init(std::filesystem::path vessel_config) {
    const VesselParams p = vessel_params_;

    // Compose init state
    eta_ = p.eta_0;
    nu_ = p.nu_0;
    x_.subvec(0, 5) = eta_;
    x_.subvec(6, 11) = nu_;

    // Construct inertia dyadic in r_cg
    const arma::mat33 I_cg = common::I_cg(p.m, p.r44, p.r55, p.r66);
    I_co_ = I_cg - p.m * common::S(p.r_cg) * common::S(p.r_cg);

    // Compose M_rb
    M_rb_ = common::M_rb(p.m, I_co_, p.r_cg);
    M_a_ = arma::mat66(arma::fill::zeros);
    M_ = M_rb_ + M_a_;

    // Compose G 
    arma::vec3 r_p = {0, 0, 0};           
    G_ = common::G(p.nabla, p.area_wp, p.gmt, p.gml, p.lcf, r_p);

    // Damping linear
    D_l_ = common::D_l(p.d_l_coeffs);

    UpdateDynamicMatrices();
}

// Update the state dependent matrices
void Dynamics::UpdateDynamicMatrices() {
    const VesselParams p = vessel_params_;

    J_ = common::J(eta_);
    C_rb_ = common::C_rb(p.m, I_co_, p.r_cg, nu_);
    g_ = common::g(p.w, p.b, p.r_cg, p.r_cb, eta_);
    D_n_ = common::D_n(p.d_n_coeffs, nu_);
    D_ = D_l_ + D_n_;
}

void Dynamics::SetTau(const arma::vec6& tau) {
    tau_ = tau;
}

} // namespace vessel