#pragma once
// ============================================================================
// Model parts that move with a csv column, e.g. the rudder with delta_r.
//
// A part is a named group in a model file: an element id (or the id of a
// parent <g>) in the SVG, an `o`/`g` name in the OBJ. Parts not listed here
// are drawn fixed to the hull.
//
// A moving part rotates about the body z axis (down) through its pivot, by
// the column value in radians. Positive is clockwise seen from above, the
// same sign as yaw, so a positive delta_r swings the rudder's trailing edge
// to port. The pivot is the part's leading edge: largest x, middle of y
// (the SVG can set it with data-pivot="x,y").
//
// To animate something new, add a line below and name the part in the models.
// ============================================================================
#include <array>
#include <string_view>

namespace vessel_model {

struct PartMotion {
    std::string_view part;    // group name in the model files
    std::string_view column;  // csv column, angle in rad
};

inline constexpr std::array<PartMotion, 1> kPartMotions = {{
    {"rudder", "delta_r"},
}};

// nullptr when the part does not move
inline const PartMotion* FindPartMotion(std::string_view part) {
    for (const auto& motion : kPartMotions) {
        if (motion.part == part) return &motion;
    }
    return nullptr;
}

}  // namespace vessel_model
