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
    const double m = 2000;                   // mass
    const double g = 9.81;                  // eple fra tre konstanten
    const double w = m * g;                 // weight
    const double b = w;                     // buoancy
    const arma::vec3 r_cg = {5, 0, 0};      // centre of gravity
    const arma::vec3 r_cb = {0, 0, 5};      // centre of boyancy
    const double r44 = 10;                  // radius of gyration in roll
    const double r55 = 10;                  // radius of gyration in pitch
    const double r66 = 10;                  // radius of gyration in yaw

    // Construct inertia dyadic in r_cg
    const arma::mat33 I_cg = arma::diagmat(arma::vec3{{m*r44*r44, m*r55*r55, m*r66*r66}});               // Inertia about cg

    // Parallell axis theorem to construct inertia dyadic in r_co = 0
    I0_ = I_cg - m * common::Smtrx(r_cg) * common::Smtrx(r_cg);

    // Construct M_rb
    M_rb_.submat(0, 0, 2, 2) =  m * arma::eye(3, 3);
    M_rb_.submat(0, 3, 2, 5) = -m * common::Smtrx(r_cg);
    M_rb_.submat(3, 0, 5, 2) =  m * common::Smtrx(r_cg);
    M_rb_.submat(3, 3, 5, 5) =  I0_;

    // Construct C_rb
    const arma::vec3 nu2 = nu_.AttitudeRate(); 
    C_rb_.submat(0, 0, 2, 2) =  m * common::Smtrx(nu2);
    C_rb_.submat(0, 3, 2, 5) = -m * common::Smtrx(nu2) * common::Smtrx(r_cg);
    C_rb_.submat(3, 0, 5, 2) =  m * common::Smtrx(r_cg) * common::Smtrx(nu2);
    C_rb_.submat(3, 3, 5, 5) =  common::Smtrx(I0_ * nu2);



}

} // namespace vessel