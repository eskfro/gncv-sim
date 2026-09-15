#pragma once

#include <armadillo>

namespace common {

// Probably dont need this
enum class Frames : int {Body, Ned, Enu};

// Eta = positions
// Position and orientetion vector interface
class Eta { // η
public:
    void Reset() { vector_.fill(0.0); }

    const double& Psi() const { return vector_.at(5); }

    arma::vec3 Position() const { return vector_.subvec(0, 2); }
    arma::vec3 Attitude() const { return vector_.subvec(3, 5); }
    const arma::vec6& Vector() const { return vector_; }

private:
    // (0-2) : (x, y, z): (surge, sway, heave)
    // (3-5) : (phi, theta, psi) : (roll, pitch, yaw)
    arma::vec6 vector_{};
};

// Nu = rates
// Position rates and orientation rates interface
class Nu { // ν
public:
    void Reset() { vector_.fill(0.0); }
    arma::vec3 PositionRate() const { return vector_.subvec(0, 2); }
    arma::vec3 AttitudeRate() const { return vector_.subvec(3, 5); }
    const arma::vec6& Vector() const { return vector_; }
private:
    // (0-2) : (u, v, w) : (surge velocity, sway velocity, heave velocity)
    // (3-5) : (p, q, r) : (roll rate, pitch rate, yaw rate)
    arma::vec6 vector_{};
};

enum class GuidanceMode : uint8_t {HeadingHold, PositionHold, WaypointTracking};

struct Reference {
    GuidanceMode guidance_mode{};
    Eta eta_d{};
    double psi_d;
    double u_d;
};

struct ActuatorCommands {
    double delta;   // rudder
    double n;       // rpm main propulsor
    double n_tt;    // rpm tunnel thruster
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
arma::mat66 join_blocks(arma::mat33 A, arma::mat33 B, arma::mat33 C, arma::mat33 D);

// Matrices
arma::mat66 H_mat(const arma::vec3& r);
arma::mat33 R_zyx(const arma::vec3& a);
arma::mat33 S_mat(const arma::vec3& a);
arma::mat33 T_zyx(const arma::vec3& a);
arma::mat66 G_mat(
    double nabla,
    double A_wp,
    double gmt, 
    double gml,
    double x_cf,
    arma::vec3 r_p
);
arma::mat66 J(const Eta& eta);
arma::mat33 I_cg(double m, double r44, double r55, double r66);
arma::mat66 M_rb(double m, const arma::mat33& I0, const arma::vec3& r_cg);
arma::mat66 C_rb(double m, const arma::mat33& I0, const arma::vec3& r_cg, const Nu& nu);
arma::vec6 g(double w, double b, const arma::vec3& r_cg,
    const arma::vec3& r_cb, const common::Eta& eta);

} // namespace common