#pragma once
// ============================================================================
// Small value types used by the vessel models.
//
// All model coordinates are in the vessel body frame, in meters:
//   x forward, y starboard, z down (same axes as nu in src/common.hpp)
// ============================================================================
#include <algorithm>
#include <cstdint>
#include <limits>

namespace vessel_model {

struct Vec2 {
    double x{};
    double y{};
};

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

// RGBA, 0-255
struct Color {
    std::uint8_t r{255};
    std::uint8_t g{255};
    std::uint8_t b{255};
    std::uint8_t a{255};
};

// Axis aligned box. Empty until the first Add().
struct Bounds3 {
    Vec3 min{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()};
    Vec3 max{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest()};

    bool Empty() const { return min.x > max.x; }
    void Add(const Vec3& p) {
        min = {std::min(min.x, p.x), std::min(min.y, p.y), std::min(min.z, p.z)};
        max = {std::max(max.x, p.x), std::max(max.y, p.y), std::max(max.z, p.z)};
    }
    void Add(const Bounds3& b) {
        if (b.Empty()) return;
        Add(b.min);
        Add(b.max);
    }
    Vec3 Size() const {
        return Empty() ? Vec3{} : Vec3{max.x - min.x, max.y - min.y, max.z - min.z};
    }
    Vec3 Center() const {
        return Empty() ? Vec3{} : Vec3{0.5 * (min.x + max.x), 0.5 * (min.y + max.y), 0.5 * (min.z + max.z)};
    }
};

}  // namespace vessel_model
