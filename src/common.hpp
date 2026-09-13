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
    arma::vec3 PositionRate() { return vector_.subvec(0, 2); }
    arma::vec3 AttitudeRate() { return vector_.subvec(3, 5); }
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

double deg2rad(double deg);
bool inrange(double value, double range_min, double range_max);
double ssa(double angle);
arma::mat33 Smtrx(const arma::vec3& a);
arma::mat33 Rzyx(const arma::vec3& a);
arma::mat66 J(const Eta& eta);

} // namespace common