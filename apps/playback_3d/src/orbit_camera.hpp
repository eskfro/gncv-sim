#pragma once
// ============================================================================
// Camera orbiting a target point on the water.
//
//   yaw       compass direction the camera looks in [rad], 0 = looking north
//   pitch     angle below the horizon [rad], 90 deg = straight down
//   distance  from the target [m]
//
// The screen is a viewport rectangle in pixels (y down), as for the 2D camera.
// ============================================================================
#include <optional>

#include "math3d.hpp"

namespace playback3d {

struct ScreenPoint {
    double x{};
    double y{};
};

class OrbitCamera {
public:
    static constexpr double kMinPitch = 0.035;  // ~2 deg, stays above the water
    static constexpr double kMaxPitch = 1.553;  // ~89 deg
    static constexpr double kMinDistance = 2.0;
    static constexpr double kMaxDistance = 50000.0;

    void SetViewport(double left, double top, double width, double height);

    V3 Target() const { return target_; }
    void SetTarget(V3 target) { target_ = target; }
    double Yaw() const { return yaw_; }
    void SetYaw(double yaw) { yaw_ = yaw; }
    double Pitch() const { return pitch_; }
    void SetPitch(double pitch);
    double Distance() const { return distance_; }
    void SetDistance(double distance);
    double FovY() const { return fov_y_; }

    V3 Eye() const;
    // Unit vector from the eye towards the target
    V3 Forward() const;

    // Mouse drag of (dx, dy) pixels: turn around the target. Dragging right
    // turns the scene right, dragging down raises the camera.
    void Orbit(double dx, double dy);
    // Mouse drag of (dx, dy) pixels: move the target over the water
    void Pan(double dx, double dy);
    // factor > 1 moves closer
    void Zoom(double factor);
    // Look at the horizontal box [min, max] from above, at the current yaw
    void Fit(V3 min, V3 max);

    // Meters per pixel at the target distance, for sizing things on screen
    double MetersPerPixel() const;

    // Matrices. Positions sent to OpenGL are relative to `origin`, which
    // keeps float precision when far from the NED origin.
    Mat4 View(V3 origin) const;
    Mat4 Projection() const;
    double NearZ() const;
    double FarZ() const;

    // Screen position of a world point, nullopt when behind the camera
    std::optional<ScreenPoint> Project(V3 world) const;

    double Left() const { return left_; }
    double Top() const { return top_; }
    double Width() const { return width_; }
    double Height() const { return height_; }

private:
    V3 target_{};
    double yaw_{0.0};
    double pitch_{0.5};
    double distance_{200.0};
    double fov_y_{0.785};  // 45 deg
    double left_{0.0};
    double top_{0.0};
    double width_{1.0};
    double height_{1.0};
};

}  // namespace playback3d
