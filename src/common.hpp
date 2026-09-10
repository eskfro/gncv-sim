#pragma once

namespace common {

enum class Frames : int {Body, Ned, Enu};

// Vector with three float values
// Probably switch to some vector math ahh lib later tho ...
class Vec3 {
public:
    Vec3() = default;
    Vec3(double x, double y, double z) : px_(x), py_(y), pz_(z) {}

    double X() const { return px_; }
    double Y() const { return py_; }
    double Z() const { return pz_; }

private:
    double px_ = 0.0;
    double py_ = 0.0;
    double pz_ = 0.0;
};

// Position and orientetion vector
struct Eta {
    Vec3 Position() { return Vec3(x, y, z); }
    Vec3 Attitude() { return Vec3(phi, theta, psi); }

    double x;       // surge
    double y;       // sway
    double z;       // heave
    double phi;     // roll
    double theta;   // pitch
    double psi;     // yaw

};

// Position rates and orientation rates vector
struct Nu {
    Vec3 PositionRate() { return Vec3(u, v, w); }
    Vec3 AttitudeRate() { return Vec3(p, q, r); }

    double u;       // surge velocity
    double v;       // sway velocity
    double w;       // heave velocity
    double p;       // roll rate
    double q;       // pitch rate
    double r;       // yaw rate

};

double deg2rad(double deg);

} // namespace common