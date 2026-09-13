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

arma::mat33 Smtrx(const arma::vec3& a) {
    return arma::mat33{
        {  0.0,   -a(2),   a(1) },
        {  a(2),   0.0,   -a(0) },
        { -a(1),   a(0),   0.0  }
    };
}

// Rzyx: Euler angle rotation matrix R in SO(3), zyx convention
// a = [phi, theta, psi]  (roll, pitch, yaw) [rad]
arma::mat33 Rzyx(const arma::vec3& a) {
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
arma::mat33 Tzyx(const arma::vec3& a) {
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
arma::mat66 J(const common::Eta& eta) {
    const arma::vec3 attitude = eta.Attitude(); // (phi, theta, psi)

    arma::mat66 J(arma::fill::zeros);
    J.submat(0, 0, 2, 2) = Rzyx(attitude);
    J.submat(3, 3, 5, 5) = Tzyx(attitude);
    return J;
}

// [rad]
// returns signed angle in range (-pi, pi)
double ssa(double angle) {
    double wrapped = std::fmod(angle + M_PI, 2 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2 * M_PI;
    }
    return wrapped - M_PI;
}

} // namespace common