#pragma once
// ============================================================================
// Top-down camera: NED world (meters) <-> screen (pixels).
//
// North points up and east points right, so the picture matches a chart and
// the Path view of simulator_v1_plotter.
// ============================================================================

namespace playback2d {

struct NedPoint {
    double north{};
    double east{};
};

struct ScreenPoint {
    double x{};  // right
    double y{};  // down
};

class Camera {
public:
    static constexpr double kMinPixelsPerMeter = 1e-3;
    static constexpr double kMaxPixelsPerMeter = 1e3;

    // Screen rectangle the world is drawn into
    void SetViewport(double left, double top, double width, double height);

    ScreenPoint ToScreen(NedPoint p) const;
    NedPoint ToWorld(ScreenPoint s) const;

    void CenterOn(NedPoint p) { center_ = p; }
    NedPoint Center() const { return center_; }

    // Move the view by a mouse drag of (dx, dy) pixels
    void Pan(double dx, double dy);
    // Zoom by factor (> 1 zooms in) while keeping `anchor` fixed on screen
    void ZoomAt(double factor, ScreenPoint anchor);
    // Show the box [north_min, north_max] x [east_min, east_max] with a margin
    void Fit(NedPoint min, NedPoint max, double margin_px);

    double PixelsPerMeter() const { return ppm_; }
    void SetPixelsPerMeter(double ppm);

    double Left() const { return left_; }
    double Top() const { return top_; }
    double Width() const { return width_; }
    double Height() const { return height_; }

private:
    NedPoint center_{};
    double ppm_{1.0};
    double left_{0.0};
    double top_{0.0};
    double width_{1.0};
    double height_{1.0};
};

}  // namespace playback2d
