#pragma once
// ============================================================================
// Where the vessel is and how it is oriented at a playback time, and the
// matrices that place the 3D model there.
// ============================================================================
#include "math3d.hpp"
#include "simdata/recording.hpp"
#include "vessel_model/geometry.hpp"

namespace playback3d {

struct VesselPose {
    V3 position;       // NED [m]
    double phi{};      // roll [rad]
    double theta{};    // pitch [rad]
    double psi{};      // yaw [rad]
};

// Roll, pitch and heave are often too small to see, so they can be scaled up
struct MotionScale {
    double attitude{1.0};  // phi and theta
    double heave{1.0};     // z
};

// Missing columns count as zero (older csv files have no z, phi or theta)
VesselPose PoseAt(const simdata::Frame& frame, MotionScale scale = {});

// Body -> world, relative to origin, with the model scaled by size_scale
Mat4 BodyToWorld(const VesselPose& pose, V3 origin, double size_scale = 1.0);

// Rotation of a moving part about the body z axis through its pivot
Mat4 PartRotation(const vessel_model::Vec3& pivot, double angle);

}  // namespace playback3d
