#pragma once
// ============================================================================
// 2D vessel model: the top view, loaded from an SVG file (model_2d.svg).
//
// SVG coordinates are body coordinates in meters. SVG x points right and y
// down, so a file viewed in a browser shows the vessel from above with the
// bow to the right and starboard down:
//
//   SVG x = body x (forward),  SVG y = body y (starboard)
//
// Supported, which covers hand written files and plain Inkscape drawings:
//   elements    path, polygon, polyline, line, rect, circle, ellipse, g, svg
//   path data   M L H V C S Q T A Z, absolute and relative
//   transform   matrix, translate, scale, rotate, skewX, skewY
//   style       fill, stroke, stroke-width, opacity, fill-opacity,
//               stroke-opacity, display (as attributes or in style="...")
//   colors      #rgb, #rrggbb, rgb(r,g,b), none and the common color names
// Everything else (text, images, gradients, defs, ...) is skipped, with a
// warning for drawable elements. A path with several subpaths fills each one
// on its own, so holes are not cut out.
//
// Moving parts (see parts.hpp) are found by element or group id, and may set
// their pivot with data-pivot="x,y" (body coordinates).
// ============================================================================
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vessel_model/geometry.hpp"

namespace vessel_model {

struct Shape2D {
    std::string part;            // nearest id, "" when none
    std::vector<Vec2> points;    // body frame [m], curves flattened
    bool closed{true};
    std::optional<Color> fill;   // nullopt = not filled
    std::optional<Color> stroke; // nullopt = no outline
    double stroke_width{0.0};    // [m]
};

struct Model2D {
    std::vector<Shape2D> shapes;  // paint order, back to front
    std::map<std::string, Vec2, std::less<>> pivots;  // per named part
    Bounds3 bounds;               // of all shapes, z = 0
    std::string source;           // file name, or "built-in"
    std::vector<std::string> warnings;

    bool Empty() const { return shapes.empty(); }
    double Length() const { return bounds.Size().x; }
    double Breadth() const { return bounds.Size().y; }
    // Pivot of a part, origin if unknown
    Vec2 Pivot(std::string_view part) const;
};

// Both throw std::runtime_error with a readable message
Model2D LoadModel2D(const std::filesystem::path& path);
Model2D ParseSvg(std::string_view svg, const std::string& source_name);

// Simple hull outline with a rudder, for vessels without model_2d.svg
Model2D DefaultModel2D(double length, double breadth);

}  // namespace vessel_model
