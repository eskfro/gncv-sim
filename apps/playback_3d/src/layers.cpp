#include "layers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "imgui.h"

#include "simdata/channels.hpp"
#include "vessel_model/parts.hpp"

namespace playback3d {

namespace {

namespace col = simdata::col;

// Heights of the flat layers [m, z down], just above the water so they are
// not hidden by it. Later layers are drawn on top of earlier ones.
constexpr double kGridZ = -0.02;
constexpr double kOverlayZ = -0.05;

constexpr Rgba kWater = Rgb8(24, 66, 96, 215);
constexpr Rgba kGridMinor = Rgb8(255, 255, 255, 28);
constexpr Rgba kGridAxis = Rgb8(255, 255, 255, 80);
constexpr Rgba kTrackPast = Rgb8(90, 190, 255, 235);
constexpr Rgba kTrackFuture = Rgb8(90, 190, 255, 70);
constexpr Rgba kTick = Rgb8(220, 230, 240, 200);
constexpr Rgba kReference = Rgb8(255, 210, 80, 235);
constexpr Rgba kVelocity = Rgb8(110, 230, 120, 235);
constexpr Rgba kForce = Rgb8(255, 120, 220, 235);

V3 Position(const simdata::Frame& f, double z = kOverlayZ) { return {f.Get(col::kX), f.Get(col::kY), z}; }

// Point given in body coordinates (x forward, y starboard) on the water
V3 BodyToWater(V3 origin, double psi, double bx, double by) {
    const double c = std::cos(psi);
    const double s = std::sin(psi);
    return {origin.x + c * bx - s * by, origin.y + s * bx + c * by, origin.z};
}

// 1, 2, 5, 10, 20, 50, ... closest at or above `raw`
double NiceStep(double raw) {
    const double base = std::pow(10.0, std::floor(std::log10(raw)));
    for (const double m : {1.0, 2.0, 5.0}) {
        if (m * base >= raw) return m * base;
    }
    return 10.0 * base;
}

// ---------------------------------------------------------------------------

class WaterLayer : public Layer {
public:
    WaterLayer() : Layer("Water", true) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        // Large enough that the fog hides its edge
        const V3 c = ctx.camera.Target();
        const double h = 0.5 * ctx.camera.FarZ();
        Rgba color = kWater;
        color.a = opacity_;
        out.Quad({c.x - h, c.y - h, 0.0}, {c.x + h, c.y - h, 0.0}, {c.x + h, c.y + h, 0.0}, {c.x - h, c.y + h, 0.0},
                 color);
    }

    void DrawSettings() override { ImGui::SliderFloat("Opacity", &opacity_, 0.0f, 1.0f, "%.2f"); }

private:
    float opacity_{kWater.a};
};

class GridLayer : public Layer {
public:
    GridLayer() : Layer("Grid", true) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        step_ = NiceStep(kTargetSpacingPx * ctx.meters_per_pixel);
        const V3 c = ctx.camera.Target();
        const double radius = std::min(kRadiusInSteps * step_, 0.4 * ctx.camera.FarZ());
        const double x0 = std::floor((c.x - radius) / step_) * step_;
        const double y0 = std::floor((c.y - radius) / step_) * step_;
        for (int k = 0; k <= 2 * kRadiusInSteps + 1; k++) {
            const double x = x0 + k * step_;
            const double y = y0 + k * step_;
            const bool x_axis = std::abs(x) < 0.5 * step_;
            const bool y_axis = std::abs(y) < 0.5 * step_;
            out.Line({x, c.y - radius, kGridZ}, {x, c.y + radius, kGridZ}, x_axis ? kGridAxis : kGridMinor);
            out.Line({c.x - radius, y, kGridZ}, {c.x + radius, y, kGridZ}, y_axis ? kGridAxis : kGridMinor);
        }
    }

    void DrawSettings() override { ImGui::TextDisabled("Spacing %g m", step_); }

private:
    static constexpr double kTargetSpacingPx = 90.0;
    static constexpr int kRadiusInSteps = 40;
    double step_{0.0};
};

class TrackLayer : public Layer {
public:
    TrackLayer() : Layer("Track", true) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        const auto& rec = ctx.recording;
        const std::size_t now = ctx.frame.Index();
        const double mpp = ctx.meters_per_pixel;

        if (show_future_) {
            Points(ctx, now, rec.Rows() - 1, mpp);
            out.Ribbon(points_, 1.5 * mpp, kTrackFuture);
        }
        Points(ctx, 0, now, mpp);
        points_.push_back(Position(ctx.frame));
        out.Ribbon(points_, 3.0 * mpp, kTrackPast);

        if (tick_interval_ > 0) {
            const auto& t = *rec.Column(col::kT);
            const auto& xs = *rec.Column(col::kX);
            const auto& ys = *rec.Column(col::kY);
            char label[32];
            for (double tick = tick_interval_; tick <= rec.EndTime(); tick += tick_interval_) {
                if (tick < t.front()) continue;
                const std::size_t i = rec.IndexAt(tick);
                const V3 p{xs[i], ys[i], kOverlayZ};
                out.Disc(p, 3.0 * mpp, kTick, 12);
                std::snprintf(label, sizeof(label), "%.0f s", tick);
                out.Text(p, label, kTick);
            }
        }
    }

    void DrawSettings() override {
        ImGui::Checkbox("Show remaining track", &show_future_);
        ImGui::SliderInt("Time marks [s]", &tick_interval_, 0, 120, tick_interval_ == 0 ? "off" : "%d");
    }

private:
    // Rows [first, last], dropping points closer than ~1 px to the previous one
    void Points(const SceneContext& ctx, std::size_t first, std::size_t last, double mpp) {
        points_.clear();
        const auto& xs = *ctx.recording.Column(col::kX);
        const auto& ys = *ctx.recording.Column(col::kY);
        for (std::size_t i = first; i <= last && i < xs.size(); i++) {
            const V3 p{xs[i], ys[i], kOverlayZ};
            if (!points_.empty() && i != last && Length(p - points_.back()) < mpp) continue;
            points_.push_back(p);
        }
    }

    bool show_future_{true};
    int tick_interval_{20};
    std::vector<V3> points_;
};

class ReferenceLayer : public Layer {
public:
    ReferenceLayer() : Layer("Reference", true) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        const auto& f = ctx.frame;
        const double mpp = ctx.meters_per_pixel;
        const V3 pos = Position(f);

        // Desired heading psi_d as a dashed line from the ship
        if (f.Has(col::kPsiD)) {
            const double len = std::max(1.5 * ctx.ship_length, 80.0 * mpp);
            const V3 tip = BodyToWater(pos, f.Get(col::kPsiD), len, 0.0);
            out.Dashed(pos, tip, 1.5 * mpp, 8.0 * mpp, 6.0 * mpp, kReference);
            out.Text(tip, "psi_d", kReference);
        }

        // Desired position. It is all zeros in HeadingHold, so only drawn when used.
        if (f.Has(col::kXd) && f.Has(col::kYd)) {
            const double x_d = f.Get(col::kXd);
            const double y_d = f.Get(col::kYd);
            const bool heading_hold = f.Get(col::kGuidanceMode) == 0.0;
            if (!heading_hold || x_d != 0.0 || y_d != 0.0) {
                const V3 p{x_d, y_d, kOverlayZ};
                out.Ring(p, 9.0 * mpp, 2.0 * mpp, kReference);
                out.Ribbon({p + V3{13.0 * mpp, 0, 0}, p - V3{13.0 * mpp, 0, 0}}, 1.5 * mpp, kReference);
                out.Ribbon({p + V3{0, 13.0 * mpp, 0}, p - V3{0, 13.0 * mpp, 0}}, 1.5 * mpp, kReference);
                out.Text(p, "setpoint", kReference);
            }
        }
    }
};

class VelocityLayer : public Layer {
public:
    VelocityLayer() : Layer("Velocity", true) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        const auto& f = ctx.frame;
        const double u = f.Get(col::kU);
        const double v = f.Get(col::kV);
        if (std::hypot(u, v) < 1e-3) return;
        // Where the ship would be after `lookahead_` seconds at this velocity
        const V3 pos = Position(f);
        out.Arrow(pos, BodyToWater(pos, f.Get(col::kPsi), u * lookahead_, v * lookahead_),
                  2.0 * ctx.meters_per_pixel, kVelocity);
    }

    void DrawSettings() override { ImGui::SliderFloat("Look ahead [s]", &lookahead_, 1.0f, 120.0f, "%.0f"); }

private:
    float lookahead_{30.0f};
};

class ForceLayer : public Layer {
public:
    ForceLayer() : Layer("Force (tau_X, tau_Y)", false) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        const auto& f = ctx.frame;
        if (!f.Has(col::kTauX) || !f.Has(col::kTauY)) return;
        const V3 pos = Position(f);
        const double sx = f.Get(col::kTauX) / 1000.0 * meters_per_kn_;
        const double sy = f.Get(col::kTauY) / 1000.0 * meters_per_kn_;
        out.Arrow(pos, BodyToWater(pos, f.Get(col::kPsi), sx, sy), 2.0 * ctx.meters_per_pixel, kForce);
    }

    void DrawSettings() override {
        ImGui::SliderFloat("Scale [m/kN]", &meters_per_kn_, 0.01f, 10.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    }

private:
    float meters_per_kn_{0.5f};
};

// The vessel's model_3d.obj in its full 6 DOF pose, with moving parts
// (rudder) set from the csv
class VesselLayer : public Layer {
public:
    VesselLayer() : Layer("Vessel", true) {}

    void Build(const SceneContext& ctx, SceneBatch& out) override {
        // Enlarge the vessel when it would be too small to see
        const double true_px = ctx.ship_length / ctx.meters_per_pixel;
        const double k = std::max(static_cast<double>(scale_), min_length_px_ / std::max(true_px, 1e-9));

        VesselDraw draw;
        draw.body = BodyToWorld(PoseAt(ctx.frame, ctx.motion), out.Origin(), k);
        for (const auto& part : ctx.model.parts) {
            const auto* motion = vessel_model::FindPartMotion(part.name);
            draw.parts.push_back(motion != nullptr
                                     ? PartRotation(ctx.model.Pivot(part.name), ctx.frame.Get(motion->column))
                                     : Mat4{});
        }
        out.Vessel(std::move(draw));
    }

    void DrawSettings() override {
        ImGui::SliderFloat("Size", &scale_, 1.0f, 20.0f, "%.1fx", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("Min length [px]", &min_length_px_, 0.0f, 120.0f, "%.0f");
    }

private:
    float scale_{1.0f};
    float min_length_px_{40.0f};
};

}  // namespace

std::vector<std::unique_ptr<Layer>> CreateDefaultLayers() {
    std::vector<std::unique_ptr<Layer>> layers;
    layers.push_back(std::make_unique<WaterLayer>());
    layers.push_back(std::make_unique<GridLayer>());
    layers.push_back(std::make_unique<TrackLayer>());
    layers.push_back(std::make_unique<ReferenceLayer>());
    layers.push_back(std::make_unique<ForceLayer>());
    layers.push_back(std::make_unique<VelocityLayer>());
    layers.push_back(std::make_unique<VesselLayer>());
    return layers;
}

}  // namespace playback3d
