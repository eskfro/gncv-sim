#pragma once

#include <armadillo>
#include <functional>

namespace common {

// Probably dont need this
enum class Frames : int {Body, Ned, Enu};

// Custom vector and matrix definitionss
using vec12 = arma::Col<double>::fixed<12>;
using mat24 = arma::Mat<double>::fixed<2, 4>;
using mat42 = arma::Mat<double>::fixed<4, 2>;

// Eta = positions
// Position and orientetion vector interface
class Eta { // η
public:
    void Reset() { vector_.fill(0.0); }
    void SetVector(arma::vec6 vector) { vector_ = vector; }

    const double Psi() const { return vector_.at(5); }
    arma::vec3 Position() const { return vector_.subvec(0, 2); }
    arma::vec3 Attitude() const { return vector_.subvec(3, 5); }
    const arma::vec6& Vector() const { return vector_; }

private:
    // (0-2) : (x, y, z): north, east, down positons
    // (3-5) : (phi, theta, psi) : roll, pitch, yaw angle
    arma::vec6 vector_{};
};

// Nu = rates
// Position rates and orientation rates interface
class Nu { // ν
public:
    void Reset() { vector_.fill(0.0); }
    void SetVector(arma::vec6 vector) { vector_ = vector; }

    double u() { return vector_.at(0); }
    double v() { return vector_.at(1); }
    double w() { return vector_.at(2); }

    arma::vec3 LinearVelocity() const { return vector_.subvec(0, 2); }
    arma::vec3 AngularVelocity() const { return vector_.subvec(3, 5); }
    const arma::vec6& Vector() const { return vector_; }
private:
    // (0-2) : (u, v, w) : surge, sway, heave velocity
    // (3-5) : (p, q, r) : roll, pitch, yaw rates
    arma::vec6 vector_{};
};

enum class GuidanceMode : uint8_t {HeadingHold, PositionHold, WaypointTracking};

struct Reference {
    GuidanceMode guidance_mode{};
    Eta eta_d{};
    double psi_d{};
    double u_d{};
};

struct ActuatorCommands {
    double delta_r;   // rudder
    double n_mp;      // rpm main propulsor
    double n_tt;      // rpm tunnel thruster
};

struct ImuSnapshot {
    double ax;      // accelerations 
    double ay;
    double az;
    double p;       // angular rates in body frame
    double q;
    double r;
};

// Utilities
double deg2rad(double deg);
bool inrange(double value, double range_min, double range_max);
double ssa(double angle);
arma::mat66 join33blocks(arma::mat33 A, arma::mat33 B, arma::mat33 C, arma::mat33 D);
arma::mat44 join22blocks(arma::mat33 A, arma::mat33 B, arma::mat33 C, arma::mat33 D);

// Numerical solvers
vec12 solver_12d_rk4(std::function<vec12(vec12, double)> f, vec12 x, double t, double dt);
double first_order_lowpass(double dt, double time_constant, double command, double curr);

// Matrices :)
arma::mat33 R_zyx(const arma::vec3& a);
arma::mat33 T_zyx(const arma::vec3& a);
arma::mat66 H(const arma::vec3& r);
arma::mat33 S(const arma::vec3& a);
arma::mat66 G(double nabla, double A_wp, double gmt, double gml, double x_cf, arma::vec3 r_p);
arma::mat66 J(const arma::vec6& eta);
arma::mat33 I_cg(double m, double r44, double r55, double r66);
arma::mat66 M_rb(double m, const arma::mat33& I0, const arma::vec3& r_cg);
arma::mat66 C_rb(double m, const arma::mat33& I0, const arma::vec3& r_cg, const arma::vec6& nu);
arma::vec6 g(double w, double b, const arma::vec3& r_cg,
    const arma::vec3& r_cb, const arma::vec6& eta);
arma::mat66 D_n(const arma::vec6& coeffs, const arma::vec6& nu);
arma::mat66 D_l(const arma::vec6& coeffs);

} // namespace common