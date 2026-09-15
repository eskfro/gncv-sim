#include "dynamics.hpp"
#include "common.hpp"
#include <armadillo>
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

void Dynamics::Step(double dt) {
    UpdateDynamicMatrices();
}

void Dynamics::Init(std::filesystem::path vessel_config) {
    const VesselParams p = vessel_params_;

    // Compose init state
    eta_.SetVector(p.eta_0.Vector());
    nu_.SetVector(p.nu_0.Vector());
    x_.subvec(0, 5) = eta_.Vector();
    x_.subvec(6, 11) = nu_.Vector();

    // Construct inertia dyadic in r_cg
    const arma::mat33 I_cg = common::I_cg(p.m, p.r44, p.r55, p.r66); // Inertia about cg

    // Parallell axis theorem to construct inertia dyadic in r_co = 0
    I_co_ = I_cg - p.m * common::S(p.r_cg) * common::S(p.r_cg);

    // Construct M_rb
    M_rb_ = common::M_rb(p.m, I_co_, p.r_cg);

    // Construct C_rb
    C_rb_ = common::C_rb(p.m, I_co_, p.r_cg, nu_);

    // Construct G                      (G is to be used by controller but lets have it here anyways)
    arma::vec3 r_p = {0, 0, 0};           
    G_ = common::G(p.nabla, p.area_wp, p.gmt, p.gml, p.lcf, r_p);

    // Bouyancy forces
    g_ = common::g(p.w, p.b, p.r_cg, p.r_cb, eta_);

    // Damping
    D_l_ = common::D_l(p.d_l_coeffs);
    D_n_ = common::D_n(p.d_n_coeffs, nu_);
    D_ = D_l_ + D_n_;
}

// Update the state dependent matrices
void Dynamics::UpdateDynamicMatrices() {
    const VesselParams p = vessel_params_;
    C_rb_ = common::C_rb(p.m, I_co_, p.r_cg, nu_);
    g_ = common::g(p.w, p.b, p.r_cg, p.r_cb, eta_);
    D_n_ = common::D_n(p.d_n_coeffs, nu_);
    D_ = D_l_ + D_n_;
}

} // namespace vessel