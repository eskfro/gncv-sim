#pragma once
// ============================================================================
// 3D vessel model: a triangle mesh loaded from a Wavefront OBJ file
// (model_3d.obj), with colors from the MTL file it names (model_3d.mtl).
//
// OBJ coordinates are body coordinates in meters: x forward, y starboard,
// z down. When exporting from Blender, choose Forward = X and Up = -Z.
// Units are not scaled, so model the vessel at its real size.
//
// Supported: v, vn, f (any polygon, v, v/vt, v//vn and v/vt/vn, negative
// indices), o and g (part names), usemtl and mtllib (Kd color, d or Tr for
// opacity). Texture coordinates, lines and smoothing groups are ignored.
// Faces without normals are flat shaded.
//
// Moving parts (see parts.hpp) are found by o/g name. Their pivot is the
// leading edge of the part: largest x, middle of y and z.
// ============================================================================
#include <cstddef>
#include <filesystem>
#include <functional>
#include <istream>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "vessel_model/geometry.hpp"

namespace vessel_model {

// Triangle soup: three entries per triangle in each array
struct Part3D {
    std::string name;  // o/g name, "" before the first one
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;  // unit length
    std::vector<Color> colors;
    Bounds3 bounds;

    std::size_t Triangles() const { return positions.size() / 3; }
};

struct Model3D {
    std::vector<Part3D> parts;
    std::map<std::string, Vec3, std::less<>> pivots;  // per named part
    Bounds3 bounds;
    std::string source;
    std::vector<std::string> warnings;

    bool Empty() const { return Triangles() == 0; }
    std::size_t Triangles() const;
    double Length() const { return bounds.Size().x; }
    // Pivot of a part, origin if unknown
    Vec3 Pivot(std::string_view part) const;
};

using Materials = std::map<std::string, Color, std::less<>>;
// Returns the materials in an mtllib file named by the OBJ. Throws
// std::runtime_error when the file can not be read.
using MaterialLoader = std::function<Materials(const std::string& mtl_file)>;

// Reads the OBJ and the MTL files it names from the same folder.
// Throws std::runtime_error with a readable message.
Model3D LoadModel3D(const std::filesystem::path& path);
Model3D ParseObj(std::istream& obj, const std::string& source_name, const MaterialLoader& load_materials);
Materials ParseMtl(std::istream& mtl);

// Simple box hull with a pointed bow and a rudder, for vessels without
// model_3d.obj. The waterline is at z = 0.
Model3D DefaultModel3D(double length, double breadth, double draft);

}  // namespace vessel_model
