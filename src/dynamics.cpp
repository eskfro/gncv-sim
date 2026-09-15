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


}

void Dynamics::Init(std::filesystem::path vessel_config) {
    // I will make the config file structure later
    // ... first make the program work

    // Vessel params
    const double m = 2000;                  // mass
    const double g = 9.81;                  // eple fra tre konstanten
    const double w = m * g;                 // weight
    const double b = w;                     // buoancy
    const arma::vec3 r_cg = {5, 0, 0};      // centre of gravity
    const arma::vec3 r_cb = {0, 0, 5};      // centre of boyancy
    const double r44 = 10;                  // radius of gyration in roll
    const double r55 = 10;                  // radius of gyration in pitch
    const double r66 = 10;                  // radius of gyration in yaw

    // Construct inertia dyadic in r_cg
    const arma::mat33 I_cg = common::I_cg(m, r44, r55, r66); // Inertia about cg

    // Parallell axis theorem to construct inertia dyadic in r_co = 0
    I0_ = I_cg - m * common::S_mat(r_cg) * common::S_mat(r_cg);

    // Construct M_rb
    M_rb_ = common::M_rb(m, I0_, r_cg);

    // Construct C_rb
    C_rb_ = common::C_rb(m, I0_, r_cg, nu_);

    // Construct G
    // We need g(eta) in the dynamics - not G ... 
    double A_wp = 1000;                     // waterplane area
    double nabla = 10000;                   // displacement
    double gmt = 2;                         // transverse metacentric height
    double gml = 8;                         // longitudinal metacentric heights
    double lcf = -5;                        // location of the cf (centre floation) relatice to co
    arma::vec3 r_p = {0, 0, 0};             // location of cp (?) relative to co  
    G_ = common::G_mat(nabla, A_wp, gmt, gml, lcf, r_p);

    // Bouyancy forces
    g_ = common::g(w, b, r_cg, r_cb, eta_);
}

} // namespace vessel