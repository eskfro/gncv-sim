#include "vessel_pose.hpp"

#include "simdata/channels.hpp"

namespace playback3d {

namespace col = simdata::col;

VesselPose PoseAt(const simdata::Frame& f, MotionScale scale) {
    VesselPose pose;
    pose.position = {f.Get(col::kX), f.Get(col::kY), scale.heave * f.Get(col::kZ)};
    pose.phi = scale.attitude * f.Get(col::kPhi);
    pose.theta = scale.attitude * f.Get(col::kTheta);
    pose.psi = f.Get(col::kPsi);
    return pose;
}

Mat4 BodyToWorld(const VesselPose& pose, V3 origin, double size_scale) {
    return Translate(pose.position - origin) * RotZYX(pose.phi, pose.theta, pose.psi) * Scale(size_scale);
}

Mat4 PartRotation(const vessel_model::Vec3& pivot, double angle) {
    const V3 p{pivot.x, pivot.y, pivot.z};
    return Translate(p) * RotZ(angle) * Translate(p * -1.0);
}

}  // namespace playback3d
