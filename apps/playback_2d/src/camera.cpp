#include "camera.hpp"

#include <algorithm>
#include <cmath>

namespace playback2d {

void Camera::SetViewport(double left, double top, double width, double height) {
    left_ = left;
    top_ = top;
    width_ = std::max(width, 1.0);
    height_ = std::max(height, 1.0);
}

ScreenPoint Camera::ToScreen(NedPoint p) const {
    return {left_ + 0.5 * width_ + (p.east - center_.east) * ppm_,
            top_ + 0.5 * height_ - (p.north - center_.north) * ppm_};
}

NedPoint Camera::ToWorld(ScreenPoint s) const {
    return {center_.north - (s.y - top_ - 0.5 * height_) / ppm_,
            center_.east + (s.x - left_ - 0.5 * width_) / ppm_};
}

void Camera::Pan(double dx, double dy) {
    center_.east -= dx / ppm_;
    center_.north += dy / ppm_;
}

void Camera::ZoomAt(double factor, ScreenPoint anchor) {
    if (!(factor > 0.0)) return;
    const NedPoint before = ToWorld(anchor);
    SetPixelsPerMeter(ppm_ * factor);
    const NedPoint after = ToWorld(anchor);
    center_.north += before.north - after.north;
    center_.east += before.east - after.east;
}

void Camera::Fit(NedPoint min, NedPoint max, double margin_px) {
    center_ = {0.5 * (min.north + max.north), 0.5 * (min.east + max.east)};
    const double span_n = std::max(max.north - min.north, 1.0);
    const double span_e = std::max(max.east - min.east, 1.0);
    const double usable_w = std::max(width_ - 2.0 * margin_px, 1.0);
    const double usable_h = std::max(height_ - 2.0 * margin_px, 1.0);
    SetPixelsPerMeter(std::min(usable_w / span_e, usable_h / span_n));
}

void Camera::SetPixelsPerMeter(double ppm) {
    if (!std::isfinite(ppm)) return;
    ppm_ = std::clamp(ppm, kMinPixelsPerMeter, kMaxPixelsPerMeter);
}

}  // namespace playback2d
