#include "common.hpp"

#include <armadillo>
#include <cmath>
#include <math.h>

namespace common {

double deg2rad(double deg) {
    return M_PI * deg / 180.0;
}

bool inrange(double value, double range_min, double range_max) {
    /*
    Check if a value is inside a range
    Helper to make code more readable
    */
    return range_min < value && value < range_max;
}

// Smallest signed angle [rad]
// Maps angle to range (-pi, pi)
double ssa(double angle) {
    double wrapped = std::fmod(angle + M_PI, 2 * M_PI);
    if (wrapped < 0.0) wrapped += 2 * M_PI;
    return wrapped - M_PI;
}

// First order lowpass dynamics step
double first_order_lowpass(double dt, double time_constant, double command, double curr) {
    return (dt / time_constant) * (command - curr);
}

arma::mat66 join33blocks(arma::mat33 A, arma::mat33 B, arma::mat33 C, arma::mat33 D) {
    return arma::join_cols(
        arma::join_rows(A, B),
        arma::join_rows(C, D)
    );
}

arma::mat44 join22blocks(arma::mat22 A, arma::mat22 B, arma::mat22 C, arma::mat22 D) {
    return arma::join_cols(
        arma::join_rows(A, B),
        arma::join_rows(C, D)
    );
}

// Cross product matrix
// S(a)b = a x b
arma::mat33 S(const arma::vec3& a) {
    return arma::mat33{
        {  0.0,   -a(2),   a(1) },
        {  a(2),   0.0,   -a(0) },
        { -a(1),   a(0),   0.0  }
    };
}

// Spring stiffness matrix G about P for a floating vessel
// This is a linearized version of a nonlinear system, 
// but it works well for small roll and pitch angles.
// Reference: Gmtrx.m (MSS, Thor I. Fossen)
arma::mat66 G(
    double nabla,       // volume displacement
    double A_wp,        // area waterplane
    double gmt,         // transverse metacentric height [m]
    double gml,         // longitudunal metacentric height [m]
    double x_cf,        // vector from co to cf (centre flotation)
    arma::vec3 r_p      // vector from co to P
) {
    int rho = 1025;     // water density
    double g = 9.81;

    arma::vec3 r_cf = {x_cf, 0, 0};

    double g33_cf = rho * g * A_wp;
    double g44_cf = rho * g * nabla * gmt;
    double g55_cf = rho * g * nabla * gml;

    // G_cf
    arma::vec6 d = {0, 0, g33_cf, g44_cf, g55_cf, 0};
    arma::mat66 G_cf = arma::diagmat(d);

    // G_co
    // Transform it to co
    arma::mat66 G_co = arma::trans(H(r_cf)) * G_cf * H(r_cf);
    // Transform it to p
    arma::mat66 G = arma::trans(H(r_p)) * G_co * H(r_p); 

    return G;
}

// System transformation matrix
// Generalized parallel-axis theorem
// Reference: Hmtrx.m (MSS, Thor I. Fossen)
arma::mat66 H(const arma::vec3& r) {
    const arma::mat33 S_r = S(r);
    const arma::mat33 H11 = arma::eye(3, 3);
    const arma::mat33 H12 = arma::trans(S_r);
    const arma::mat33 H21 = arma::zeros(3, 3);
    const arma::mat33 H22 = arma::eye(3, 3);

    return join33blocks(H11, H12, H21, H22);
}

// Rzyx: Euler angle rotation matrix R in SO(3), zyx convention
// a = [phi, theta, psi]  (roll, pitch, yaw) [rad]
arma::mat33 R_zyx(const arma::vec3& a) {
    const double phi   = a(0);
    const double theta = a(1);
    const double psi   = a(2);

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double cth  = std::cos(theta);
    const double sth  = std::sin(theta);
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);

    return arma::mat33{
        { cpsi*cth,  -spsi*cphi + cpsi*sth*sphi,   spsi*sphi + cpsi*cphi*sth },
        { spsi*cth,   cpsi*cphi + sphi*sth*spsi,  -cpsi*sphi + sth*spsi*cphi },
        { -sth,       cth*sphi,                    cth*cphi                 }
    };
}

// Tzyx: Euler angle rate transformation matrix, zyx convention
// a = [phi, theta, psi]  (roll, pitch, yaw) [rad]
arma::mat33 T_zyx(const arma::vec3& a) {
    const double phi   = a(0);
    const double theta = a(1);

    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double cth  = std::cos(theta);
    const double sth  = std::sin(theta);

    return arma::mat33{
        { 1.0,   sphi * sth / cth,   cphi * sth / cth },
        { 0.0,   cphi,              -sphi             },
        { 0.0,   sphi / cth,         cphi / cth        }
    };
}

// J(eta) : kinematic transformation matrix
// Transforms the body frame linear/angular velocities (nu) into 
// ned framed position rates and euler angle rates (eta_dot)
arma::mat66 J(const arma::vec6& eta) {
    const arma::vec3 attitude = eta.subvec(3, 5); // (phi, theta, psi)
    arma::mat66 J(arma::fill::zeros);
    J.submat(0, 0, 2, 2) = R_zyx(attitude);
    J.submat(3, 3, 5, 5) = T_zyx(attitude);
    return J;
}

// Inertia dyadic
arma::mat33 I_cg(double m, double r44, double r55, double r66) {
    return arma::diagmat(arma::vec3{{m*r44*r44, m*r55*r55, m*r66*r66}});
}

// Mass rigid body
arma::mat66 M_rb(double m, const arma::mat33& I0, const arma::vec3& r_cg) {
    const arma::mat33 M11 = m * arma::eye(3, 3);
    const arma::mat33 M12 = -m * S(r_cg);
    const arma::mat33 M21 = m * S(r_cg);
    const arma::mat33 M22 = I0;

    return join33blocks(M11, M12, M21, M22);
}

// Corelois rigid body
arma::mat66 C_rb(double m, const arma::mat33& I0, const arma::vec3& r_cg, const arma::vec6& nu) {
    const arma::vec3 nu2 = nu.subvec(3, 5);
    const arma::mat33 C11 = m * S(nu2);
    const arma::mat33 C12 = -m * S(nu2) * S(r_cg);
    const arma::mat33 C21 = m * S(r_cg) * S(nu2);
    const arma::mat33 C22 = - S(I0 * nu2);

    return join33blocks(C11, C12, C21, C22);
}

// Boyancy force (nonlinear)
arma::vec6 g(double w, double b, const arma::vec3& r_cg,
    const arma::vec3& r_cb, const arma::vec6& eta) {
    const double phi = eta(3);
    const double theta = eta(4);
    const double xg = r_cg(0); 
    const double yg = r_cg(1); 
    const double zg = r_cg(2);
    const double xb = r_cb(0);
    const double yb = r_cb(1);
    const double zb = r_cb(2);
    
    arma::vec6 g;
    g(0) =  (w-b) * std::sin(theta);
    g(1) = -(w-b) * std::cos(theta) * std::sin(phi);
    g(2) = -(w-b) * std::cos(theta) * std::cos(phi);

    g(3) = -(yg*w-yb*b) * std::cos(theta) * std::cos(phi) +
            (zg*w-zb*b) * std::cos(theta) * std::sin(phi);

    g(4) =  (zg*w-zb*b) * std::sin(theta) +
            (xg*w-xb*b) * std::cos(theta) * std::cos(phi);

    g(5) = -(xg*w-xb*b) * std::cos(theta) * std::sin(phi) +
           -(yg*w-yb*b) * std::sin(theta);

    return g;
}

vec12 solver_12d_rk4(std::function<vec12(vec12, double)> f, vec12 x, double t, double dt){
    const vec12 k1 = f(x + (0.0) * dt     ,    t + (0.0) * dt);
    const vec12 k2 = f(x + (0.5) * dt * k1,    t + (0.5) * dt);
    const vec12 k3 = f(x + (0.5) * dt * k2,    t + (0.5) * dt);
    const vec12 k4 = f(x + (1.0) * dt * k3,    t + (1.0) * dt);

    vec12 x_new = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    return x_new;
}

// D_n(nu): nonlinear (quadratic) damping matrix, diagonal-only simplification
// coeffs = [X_u|u|, Y_v|v|, Z_w|w|, K_p|p|, M_q|q|, N_r|r|]  (negative values, from ship data)
arma::mat66 D_n(const arma::vec6& coeffs, const arma::vec6& nu) {
    arma::vec6 diag_terms;
    for (int i = 0; i < 6; ++i) {
        diag_terms(i) = -coeffs(i) * std::abs(nu(i));
    }
    return arma::diagmat(diag_terms);
}

arma::mat66 D_l(const arma::vec6 &coeffs) {
    return arma::diagmat(coeffs);
}

} // namespace common