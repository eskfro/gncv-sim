#pragma once
// ============================================================================
// What the 3D view draws in one frame, built by the layers and drawn by the
// renderer. GUI and OpenGL free, so it can be unit tested.
//
// Everything is given in world coordinates (NED, meters) and stored relative
// to an origin near the camera, so floats keep their precision.
// ============================================================================
#include <optional>
#include <string>
#include <vector>

#include "math3d.hpp"

namespace playback3d {

struct Rgba {
    float r{1.0f};
    float g{1.0f};
    float b{1.0f};
    float a{1.0f};
};

// 0-255 channels, like IM_COL32
constexpr Rgba Rgb8(int r, int g, int b, int a = 255) {
    return {r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f};
}

// Position relative to the batch origin, and color
struct Vertex {
    float x, y, z;
    float r, g, b, a;
};

struct Label {
    V3 world;
    std::string text;
    Rgba color;
};

// The vessel mesh and where to draw it
struct VesselDraw {
    Mat4 body;                // body -> world, relative to the batch origin
    std::vector<Mat4> parts;  // per model part (same order), applied before body
};

class SceneBatch {
public:
    void Clear(V3 origin);
    V3 Origin() const { return origin_; }

    void Line(V3 a, V3 b, Rgba color);
    void Triangle(V3 a, V3 b, V3 c, Rgba color);
    void Quad(V3 a, V3 b, V3 c, V3 d, Rgba color);

    // Flat shapes for drawing on the water. Widths and radii in meters.
    // A thick polyline with mitred corners, lying in the horizontal plane
    void Ribbon(const std::vector<V3>& points, double width, Rgba color);
    void Disc(V3 center, double radius, Rgba color, int segments = 16);
    void Ring(V3 center, double radius, double width, Rgba color, int segments = 48);
    void Arrow(V3 from, V3 to, double width, Rgba color);
    void Dashed(V3 from, V3 to, double width, double dash, double gap, Rgba color);

    // Text drawn on top of everything at the screen position of `at`
    void Text(V3 at, std::string text, Rgba color);
    void Vessel(VesselDraw vessel) { vessel_ = std::move(vessel); }

    const std::vector<Vertex>& Triangles() const { return triangles_; }
    const std::vector<Vertex>& Lines() const { return lines_; }
    const std::vector<Label>& Labels() const { return labels_; }
    const std::optional<VesselDraw>& GetVessel() const { return vessel_; }

private:
    Vertex Make(V3 p, Rgba c) const;

    V3 origin_{};
    std::vector<Vertex> triangles_;  // drawn blended, after the vessel
    std::vector<Vertex> lines_;      // drawn last
    std::vector<Label> labels_;
    std::optional<VesselDraw> vessel_;
};

}  // namespace playback3d
