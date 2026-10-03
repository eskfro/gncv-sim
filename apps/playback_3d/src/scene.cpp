#include "scene.hpp"

#include <algorithm>
#include <cmath>

namespace playback3d {

namespace {

constexpr double kMaxMiter = 2.5;  // corner length limit, in half widths

// Horizontal unit normal of the segment a -> b (to its right seen from above)
V3 SideNormal(V3 a, V3 b) { return Normalize(V3{-(b.y - a.y), b.x - a.x, 0.0}); }

}  // namespace

void SceneBatch::Clear(V3 origin) {
    origin_ = origin;
    triangles_.clear();
    lines_.clear();
    labels_.clear();
    vessel_.reset();
}

Vertex SceneBatch::Make(V3 p, Rgba c) const {
    const V3 r = p - origin_;
    return {static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z), c.r, c.g, c.b, c.a};
}

void SceneBatch::Line(V3 a, V3 b, Rgba color) {
    lines_.push_back(Make(a, color));
    lines_.push_back(Make(b, color));
}

void SceneBatch::Triangle(V3 a, V3 b, V3 c, Rgba color) {
    triangles_.push_back(Make(a, color));
    triangles_.push_back(Make(b, color));
    triangles_.push_back(Make(c, color));
}

void SceneBatch::Quad(V3 a, V3 b, V3 c, V3 d, Rgba color) {
    Triangle(a, b, c, color);
    Triangle(a, c, d, color);
}

void SceneBatch::Ribbon(const std::vector<V3>& points, double width, Rgba color) {
    if (points.size() < 2 || !(width > 0.0)) return;
    const double half = 0.5 * width;
    const std::size_t n = points.size();

    // Offset direction at each point: the mitred average of its segments
    std::vector<V3> offsets(n);
    for (std::size_t i = 0; i < n; i++) {
        const V3 before = i > 0 ? SideNormal(points[i - 1], points[i]) : V3{};
        const V3 after = i + 1 < n ? SideNormal(points[i], points[i + 1]) : V3{};
        V3 dir = Normalize(before + after);
        if (Length(dir) == 0.0) dir = Length(after) > 0.0 ? after : before;  // ends, or a full turn
        const V3 seg = Length(before) > 0.0 ? before : after;
        const double cos_half = std::max(Dot(dir, seg), 1.0 / kMaxMiter);
        offsets[i] = dir * (half / cos_half);
    }
    for (std::size_t i = 0; i + 1 < n; i++) {
        if (Length(points[i + 1] - points[i]) == 0.0) continue;
        Quad(points[i] - offsets[i], points[i + 1] - offsets[i + 1], points[i + 1] + offsets[i + 1],
             points[i] + offsets[i], color);
    }
}

void SceneBatch::Disc(V3 center, double radius, Rgba color, int segments) {
    if (!(radius > 0.0) || segments < 3) return;
    for (int k = 0; k < segments; k++) {
        const double a0 = 2.0 * M_PI * k / segments;
        const double a1 = 2.0 * M_PI * (k + 1) / segments;
        Triangle(center, center + V3{radius * std::cos(a0), radius * std::sin(a0), 0.0},
                 center + V3{radius * std::cos(a1), radius * std::sin(a1), 0.0}, color);
    }
}

void SceneBatch::Ring(V3 center, double radius, double width, Rgba color, int segments) {
    if (!(radius > 0.0) || !(width > 0.0) || segments < 3) return;
    const double r0 = std::max(radius - 0.5 * width, 0.0);
    const double r1 = radius + 0.5 * width;
    for (int k = 0; k < segments; k++) {
        const double a0 = 2.0 * M_PI * k / segments;
        const double a1 = 2.0 * M_PI * (k + 1) / segments;
        const V3 d0{std::cos(a0), std::sin(a0), 0.0};
        const V3 d1{std::cos(a1), std::sin(a1), 0.0};
        Quad(center + d0 * r0, center + d1 * r0, center + d1 * r1, center + d0 * r1, color);
    }
}

void SceneBatch::Arrow(V3 from, V3 to, double width, Rgba color) {
    const double len = Length(to - from);
    if (len < 1e-9 || !(width > 0.0)) return;
    const V3 dir = (to - from) * (1.0 / len);
    const V3 side = SideNormal(from, to);
    const double head = std::min(4.0 * width, 0.4 * len);
    const V3 base = to - dir * head;
    Ribbon({from, base}, width, color);
    Triangle(to, base + side * (1.5 * width), base - side * (1.5 * width), color);
}

void SceneBatch::Dashed(V3 from, V3 to, double width, double dash, double gap, Rgba color) {
    const double len = Length(to - from);
    if (len < 1e-9 || !(dash > 0.0) || gap < 0.0) return;
    const V3 dir = (to - from) * (1.0 / len);
    for (double s = 0.0; s < len; s += dash + gap) {
        Ribbon({from + dir * s, from + dir * std::min(s + dash, len)}, width, color);
    }
}

void SceneBatch::Text(V3 at, std::string text, Rgba color) { labels_.push_back({at, std::move(text), color}); }

}  // namespace playback3d
