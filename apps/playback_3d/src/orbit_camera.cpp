#include "orbit_camera.hpp"

#include <algorithm>

namespace playback3d {

namespace {

constexpr double kOrbitRadPerPixel = 0.005;
constexpr double kFitMargin = 1.15;
constexpr double kFitPitch = 1.05;  // ~60 deg

}  // namespace

void OrbitCamera::SetViewport(double left, double top, double width, double height) {
    left_ = left;
    top_ = top;
    width_ = std::max(width, 1.0);
    height_ = std::max(height, 1.0);
}

void OrbitCamera::SetPitch(double pitch) {
    if (std::isfinite(pitch)) pitch_ = std::clamp(pitch, kMinPitch, kMaxPitch);
}

void OrbitCamera::SetDistance(double distance) {
    if (std::isfinite(distance)) distance_ = std::clamp(distance, kMinDistance, kMaxDistance);
}

V3 OrbitCamera::Forward() const {
    // z down, so a positive pitch looks down
    return {std::cos(pitch_) * std::cos(yaw_), std::cos(pitch_) * std::sin(yaw_), std::sin(pitch_)};
}

V3 OrbitCamera::Eye() const { return target_ - Forward() * distance_; }

void OrbitCamera::Orbit(double dx, double dy) {
    // Grab the scene: dragging right turns it right
    yaw_ = std::remainder(yaw_ - dx * kOrbitRadPerPixel, 2.0 * M_PI);
    SetPitch(pitch_ + dy * kOrbitRadPerPixel);
}

void OrbitCamera::Pan(double dx, double dy) {
    // Grab the water: dragging right moves the world right
    const double k = MetersPerPixel();
    const V3 right{-std::sin(yaw_), std::cos(yaw_), 0.0};
    const V3 ahead{std::cos(yaw_), std::sin(yaw_), 0.0};
    target_ = target_ - right * (dx * k) + ahead * (dy * k / std::max(std::sin(pitch_), 0.2));
}

void OrbitCamera::Zoom(double factor) {
    if (factor > 0.0) SetDistance(distance_ / factor);
}

void OrbitCamera::Fit(V3 min, V3 max) {
    target_ = {0.5 * (min.x + max.x), 0.5 * (min.y + max.y), 0.0};
    const double half_span = 0.5 * std::max({max.x - min.x, max.y - min.y, 1.0});
    const double aspect = width_ / height_;
    const double half_fov = 0.5 * fov_y_ * std::min(aspect, 1.0);
    SetPitch(kFitPitch);
    SetDistance(kFitMargin * half_span / std::tan(half_fov));
}

double OrbitCamera::MetersPerPixel() const { return 2.0 * distance_ * std::tan(0.5 * fov_y_) / height_; }

Mat4 OrbitCamera::View(V3 origin) const { return LookAt(Eye() - origin, target_ - origin, {0.0, 0.0, -1.0}); }

double OrbitCamera::NearZ() const { return std::max(0.1, 0.01 * distance_); }

double OrbitCamera::FarZ() const { return 60.0 * distance_ + 5000.0; }

Mat4 OrbitCamera::Projection() const { return Perspective(fov_y_, width_ / height_, NearZ(), FarZ()); }

std::optional<ScreenPoint> OrbitCamera::Project(V3 world) const {
    const auto clip = TransformClip(Projection() * View(target_), world - target_);
    if (clip[3] <= 1e-9) return std::nullopt;
    const double nx = clip[0] / clip[3];
    const double ny = clip[1] / clip[3];
    return ScreenPoint{left_ + (0.5 * nx + 0.5) * width_, top_ + (0.5 - 0.5 * ny) * height_};
}

}  // namespace playback3d
