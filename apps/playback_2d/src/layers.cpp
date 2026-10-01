#include "layers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace playback2d {

namespace {

constexpr ImU32 kGridMinor = IM_COL32(255, 255, 255, 18);
constexpr ImU32 kGridAxis = IM_COL32(255, 255, 255, 60);
constexpr ImU32 kGridLabel = IM_COL32(200, 215, 230, 150);
constexpr ImU32 kTrackPast = IM_COL32(90, 190, 255, 230);
constexpr ImU32 kTrackFuture = IM_COL32(90, 190, 255, 55);
constexpr ImU32 kTick = IM_COL32(220, 230, 240, 140);
constexpr ImU32 kReference = IM_COL32(255, 210, 80, 230);
constexpr ImU32 kHullFill = IM_COL32(235, 235, 225, 255);
constexpr ImU32 kHullEdge = IM_COL32(30, 30, 30, 255);
constexpr ImU32 kBridge = IM_COL32(120, 130, 140, 255);
constexpr ImU32 kRudder = IM_COL32(255, 90, 70, 255);
constexpr ImU32 kVelocity = IM_COL32(110, 230, 120, 230);
constexpr ImU32 kForce = IM_COL32(255, 120, 220, 230);

ImVec2 ToIm(ScreenPoint p) { return {static_cast<float>(p.x), static_cast<float>(p.y)}; }

ImVec2 WorldToIm(const Camera& cam, NedPoint p) { return ToIm(cam.ToScreen(p)); }

// Point given in body coordinates (x forward, y starboard) relative to origin
NedPoint BodyToNed(NedPoint origin, double psi, double bx, double by) {
    const double c = std::cos(psi);
    const double s = std::sin(psi);
    return {origin.north + c * bx - s * by, origin.east + s * bx + c * by};
}

NedPoint Position(const Frame& f) { return {f.Get(col::kX), f.Get(col::kY)}; }

float Length(ImVec2 a, ImVec2 b) { return std::hypot(b.x - a.x, b.y - a.y); }

void Arrow(ImDrawList* dl, ImVec2 from, ImVec2 to, ImU32 color, float thickness) {
    const float len = Length(from, to);
    if (len < 1.0f) return;
    const ImVec2 dir{(to.x - from.x) / len, (to.y - from.y) / len};
    const float head = std::min(10.0f, 0.4f * len);
    const ImVec2 base{to.x - dir.x * head, to.y - dir.y * head};
    const ImVec2 side{-dir.y * head * 0.5f, dir.x * head * 0.5f};
    dl->AddLine(from, base, color, thickness);
    dl->AddTriangleFilled(to, {base.x + side.x, base.y + side.y}, {base.x - side.x, base.y - side.y},
                          color);
}

void DashedLine(ImDrawList* dl, ImVec2 from, ImVec2 to, ImU32 color, float thickness) {
    constexpr float kDash = 8.0f;
    constexpr float kGap = 6.0f;
    const float len = Length(from, to);
    if (len < 1.0f) return;
    const ImVec2 dir{(to.x - from.x) / len, (to.y - from.y) / len};
    for (float s = 0.0f; s < len; s += kDash + kGap) {
        const float e = std::min(s + kDash, len);
        dl->AddLine({from.x + dir.x * s, from.y + dir.y * s}, {from.x + dir.x * e, from.y + dir.y * e},
                    color, thickness);
    }
}

// Rows [first, last] of the track as screen points, dropping points closer
// than ~1 px to the previous one so long recordings stay cheap to draw
void TrackPoints(const DrawContext& ctx, std::size_t first, std::size_t last,
                 std::vector<ImVec2>& out) {
    out.clear();
    const auto& xs = *ctx.recording.Column(col::kX);
    const auto& ys = *ctx.recording.Column(col::kY);
    for (std::size_t i = first; i <= last && i < xs.size(); i++) {
        const ImVec2 p = WorldToIm(ctx.camera, {xs[i], ys[i]});
        if (!out.empty() && i != last && Length(out.back(), p) < 1.0f) continue;
        out.push_back(p);
    }
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

class GridLayer : public Layer {
public:
    GridLayer() : Layer("Grid", true) {}

    void Draw(const DrawContext& ctx) override {
        const Camera& cam = ctx.camera;
        const double step = NiceStep(kTargetSpacingPx / cam.PixelsPerMeter());
        const NedPoint top_left = cam.ToWorld({cam.Left(), cam.Top()});
        const NedPoint bottom_right = cam.ToWorld({cam.Left() + cam.Width(), cam.Top() + cam.Height()});
        const float left = static_cast<float>(cam.Left());
        const float top = static_cast<float>(cam.Top());
        const float right = static_cast<float>(cam.Left() + cam.Width());
        const float bottom = static_cast<float>(cam.Top() + cam.Height());

        char label[32];
        // Lines of constant east (vertical), labelled along the bottom
        for (int k = 0; k < kMaxLines; k++) {
            const double e = (std::ceil(top_left.east / step) + k) * step;
            if (e > bottom_right.east) break;
            const float x = WorldToIm(cam, {0.0, e}).x;
            const bool axis = std::abs(e) < 0.5 * step;
            ctx.draw_list->AddLine({x, top}, {x, bottom}, axis ? kGridAxis : kGridMinor);
            std::snprintf(label, sizeof(label), "%g", axis ? 0.0 : e);
            ctx.draw_list->AddText({x + 3.0f, bottom - 18.0f}, kGridLabel, label);
        }
        // Lines of constant north (horizontal), labelled along the left edge
        for (int k = 0; k < kMaxLines; k++) {
            const double n = (std::ceil(bottom_right.north / step) + k) * step;
            if (n > top_left.north) break;
            const float y = WorldToIm(cam, {n, 0.0}).y;
            const bool axis = std::abs(n) < 0.5 * step;
            ctx.draw_list->AddLine({left, y}, {right, y}, axis ? kGridAxis : kGridMinor);
            std::snprintf(label, sizeof(label), "%g", axis ? 0.0 : n);
            ctx.draw_list->AddText({left + 4.0f, y - 16.0f}, kGridLabel, label);
        }
        std::snprintf(label, sizeof(label), "grid %g m", step);
        ctx.draw_list->AddText({right - 90.0f, bottom - 40.0f}, kGridLabel, label);
    }

private:
    static constexpr double kTargetSpacingPx = 90.0;
    static constexpr int kMaxLines = 200;
};

class TrackLayer : public Layer {
public:
    TrackLayer() : Layer("Track", true) {}

    void Draw(const DrawContext& ctx) override {
        const std::size_t now = ctx.frame.Index();
        const ImVec2 current = WorldToIm(ctx.camera, Position(ctx.frame));

        if (show_future_) {
            TrackPoints(ctx, now, ctx.recording.Rows() - 1, points_);
            ctx.draw_list->AddPolyline(points_.data(), static_cast<int>(points_.size()), kTrackFuture,
                                       ImDrawFlags_None, 1.5f);
        }

        TrackPoints(ctx, 0, now, points_);
        points_.push_back(current);
        ctx.draw_list->AddPolyline(points_.data(), static_cast<int>(points_.size()), kTrackPast,
                                   ImDrawFlags_None, 2.0f);

        if (tick_interval_ > 0) {
            char label[32];
            const auto& t = *ctx.recording.Column(col::kT);
            const auto& xs = *ctx.recording.Column(col::kX);
            const auto& ys = *ctx.recording.Column(col::kY);
            for (double tick = tick_interval_; tick <= ctx.recording.EndTime(); tick += tick_interval_) {
                if (tick < t.front()) continue;
                const std::size_t i = ctx.recording.IndexAt(tick);
                const ImVec2 p = WorldToIm(ctx.camera, {xs[i], ys[i]});
                ctx.draw_list->AddCircleFilled(p, 2.5f, kTick);
                std::snprintf(label, sizeof(label), "%.0f s", tick);
                ctx.draw_list->AddText({p.x + 5.0f, p.y + 2.0f}, kTick, label);
            }
        }
    }

    void DrawSettings() override {
        ImGui::Checkbox("Show remaining track", &show_future_);
        ImGui::SliderInt("Time marks [s]", &tick_interval_, 0, 120, tick_interval_ == 0 ? "off" : "%d");
    }

private:
    bool show_future_{true};
    int tick_interval_{20};
    std::vector<ImVec2> points_;
};

class ReferenceLayer : public Layer {
public:
    ReferenceLayer() : Layer("Reference", true) {}

    void Draw(const DrawContext& ctx) override {
        const Frame& f = ctx.frame;
        const NedPoint pos = Position(f);
        const ImVec2 ship = WorldToIm(ctx.camera, pos);

        // Desired heading psi_d as a dashed line from the ship
        if (f.Has(col::kPsiD)) {
            const double len_m = std::max(1.5 * ctx.ship.length, 80.0 / ctx.camera.PixelsPerMeter());
            const ImVec2 tip = WorldToIm(ctx.camera, BodyToNed(pos, f.Get(col::kPsiD), len_m, 0.0));
            DashedLine(ctx.draw_list, ship, tip, kReference, 1.5f);
            ctx.draw_list->AddText({tip.x + 4.0f, tip.y - 8.0f}, kReference, "psi_d");
        }

        // Desired position. It is all zeros in HeadingHold, so only drawn when used.
        if (f.Has(col::kXd) && f.Has(col::kYd)) {
            const double x_d = f.Get(col::kXd);
            const double y_d = f.Get(col::kYd);
            const bool heading_hold = f.Get(col::kGuidanceMode) == 0.0;
            if (!heading_hold || x_d != 0.0 || y_d != 0.0) {
                const ImVec2 p = WorldToIm(ctx.camera, {x_d, y_d});
                ctx.draw_list->AddCircle(p, 9.0f, kReference, 0, 2.0f);
                ctx.draw_list->AddLine({p.x - 13.0f, p.y}, {p.x + 13.0f, p.y}, kReference, 1.5f);
                ctx.draw_list->AddLine({p.x, p.y - 13.0f}, {p.x, p.y + 13.0f}, kReference, 1.5f);
                ctx.draw_list->AddText({p.x + 12.0f, p.y + 6.0f}, kReference, "setpoint");
            }
        }
    }
};

class ShipLayer : public Layer {
public:
    ShipLayer() : Layer("Ship", true) {}

    void Draw(const DrawContext& ctx) override {
        const Frame& f = ctx.frame;
        const NedPoint pos = Position(f);
        const double psi = f.Get(col::kPsi);

        // Exaggerate the hull when it would be too small to see
        const double true_px = ctx.ship.length * ctx.camera.PixelsPerMeter();
        const double k = std::max(static_cast<double>(scale_), min_length_px_ / true_px);
        const double l = 0.5 * ctx.ship.length * k;  // half length
        const double b = 0.5 * ctx.ship.breadth * k;  // half breadth

        auto at = [&](double bx, double by) {
            return WorldToIm(ctx.camera, BodyToNed(pos, psi, bx, by));
        };

        // Clockwise on screen: bow, starboard side, stern, port side
        const ImVec2 hull[] = {at(l, 0.0), at(0.45 * l, b), at(-l, b), at(-l, -b), at(0.45 * l, -b)};
        ctx.draw_list->AddConvexPolyFilled(hull, 5, kHullFill);
        ctx.draw_list->AddPolyline(hull, 5, kHullEdge, ImDrawFlags_Closed, 1.5f);

        const ImVec2 bridge[] = {at(-0.35 * l, 0.7 * b), at(-0.65 * l, 0.7 * b), at(-0.65 * l, -0.7 * b),
                                 at(-0.35 * l, -0.7 * b)};
        ctx.draw_list->AddConvexPolyFilled(bridge, 4, kBridge);

        // Rudder: positive delta_r gives a starboard force at the stern
        // (F = k_r * delta_r * u^2), so the trailing edge points to port
        if (f.Has(col::kDeltaR)) {
            const double dr = f.Get(col::kDeltaR);
            const double len = 0.18 * l;
            ctx.draw_list->AddLine(at(-l, 0.0), at(-l - len * std::cos(dr), -len * std::sin(dr)), kRudder,
                                   3.0f);
        }
        ctx.draw_list->AddCircleFilled(WorldToIm(ctx.camera, pos), 2.5f, kHullEdge);
    }

    void DrawSettings() override {
        ImGui::SliderFloat("Size", &scale_, 1.0f, 20.0f, "%.1fx", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("Min length [px]", &min_length_px_, 0.0f, 120.0f, "%.0f");
    }

private:
    float scale_{1.0f};
    float min_length_px_{48.0f};
};

class VelocityLayer : public Layer {
public:
    VelocityLayer() : Layer("Velocity", true) {}

    void Draw(const DrawContext& ctx) override {
        const Frame& f = ctx.frame;
        const NedPoint pos = Position(f);
        const double u = f.Get(col::kU);
        const double v = f.Get(col::kV);
        if (std::hypot(u, v) < 1e-3) return;

        // Where the ship would be after `lookahead_` seconds at this velocity
        const NedPoint tip =
            BodyToNed(pos, f.Get(col::kPsi), u * lookahead_, v * lookahead_);
        Arrow(ctx.draw_list, WorldToIm(ctx.camera, pos), WorldToIm(ctx.camera, tip), kVelocity, 2.0f);
    }

    void DrawSettings() override {
        ImGui::SliderFloat("Look ahead [s]", &lookahead_, 1.0f, 120.0f, "%.0f");
    }

private:
    float lookahead_{30.0f};
};

class ForceLayer : public Layer {
public:
    ForceLayer() : Layer("Force (tau_X, tau_Y)", false) {}

    void Draw(const DrawContext& ctx) override {
        const Frame& f = ctx.frame;
        if (!f.Has(col::kTauX) || !f.Has(col::kTauY)) return;
        const NedPoint pos = Position(f);
        const double sx = f.Get(col::kTauX) / 1000.0 * meters_per_kn_;
        const double sy = f.Get(col::kTauY) / 1000.0 * meters_per_kn_;
        const NedPoint tip = BodyToNed(pos, f.Get(col::kPsi), sx, sy);
        Arrow(ctx.draw_list, WorldToIm(ctx.camera, pos), WorldToIm(ctx.camera, tip), kForce, 2.0f);
    }

    void DrawSettings() override {
        ImGui::SliderFloat("Scale [m/kN]", &meters_per_kn_, 0.01f, 10.0f, "%.2f",
                           ImGuiSliderFlags_Logarithmic);
    }

private:
    float meters_per_kn_{0.5f};
};

}  // namespace

std::vector<std::unique_ptr<Layer>> CreateDefaultLayers() {
    std::vector<std::unique_ptr<Layer>> layers;
    layers.push_back(std::make_unique<GridLayer>());
    layers.push_back(std::make_unique<TrackLayer>());
    layers.push_back(std::make_unique<ReferenceLayer>());
    layers.push_back(std::make_unique<ForceLayer>());
    layers.push_back(std::make_unique<VelocityLayer>());
    layers.push_back(std::make_unique<ShipLayer>());
    return layers;
}

}  // namespace playback2d
