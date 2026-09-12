#pragma once

#include <armadillo>

namespace common {

enum class Frames : int {Body, Ned, Enu};

// Position and orientetion vector interface
class Eta { // η
    arma::vec3 Position() { return vector_.subvec(0, 2); }
    arma::vec3 Attitude() { return vector_.subvec(3, 5); }
    arma::vec6& Vector() { return vector_; }

private:
    // (0-2) : (x, y, z): (surge, sway, heave)
    // (3-5) : (phi, theta, psi) : (roll, pitch, yaw)
    arma::vec6 vector_{};
};

// Position rates and orientation rates interface
class Nu { // ν
public:
    arma::vec3 PositionRate() { return vector_.subvec(0, 2); }
    arma::vec3 AttitudeRate() { return vector_.subvec(3, 5); }
    arma::vec6& Vector() { return vector_; }
private:
    // (0-2) : (u, v, w) : (surge velocity, sway velocity, heave velocity)
    // (3-5) : (p, q, r) : (roll rate, pitch rate, yaw rate)
    arma::vec6 vector_{};
};

double deg2rad(double deg);

} // namespace common