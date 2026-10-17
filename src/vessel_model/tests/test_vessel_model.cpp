#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "testing/check.hpp"
#include "vessel_model/model_2d.hpp"
#include "vessel_model/model_3d.hpp"
#include "vessel_model/parts.hpp"
#include "vessel_model/vessel_files.hpp"

using namespace vessel_model;
using testing::AlmostEqual;

namespace {

bool SvgThrows(const std::string& svg) {
    try {
        ParseSvg(svg, "test.svg");
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}

void TestSvgShapes() {
    const Model2D m = ParseSvg(R"svg(<?xml version="1.0"?>
<!-- comment with <tags> -->
<svg xmlns="http://www.w3.org/2000/svg" id="root" viewBox="-10 -5 20 10">
  <title>ignored</title>
  <defs><rect width="99" height="99"/></defs>
  <polygon points="10,0 -10,5 -10,-5" fill="#fff" stroke="black" stroke-width="0.5"/>
  <rect x="-4" y="-1" width="2" height="2" style="fill:#102030;stroke:none"/>
  <g fill="red" transform="translate(1,2)">
    <circle cx="0" cy="0" r="1"/>
  </g>
  <line x1="0" y1="0" x2="5" y2="0" stroke="blue"/>
  <rect width="1" height="1" fill="none"/>
  <text>skipped</text>
</svg>)svg",
                               "test.svg");

    CHECK(m.shapes.size() == 4);  // the unfilled, unstroked rect is dropped
    if (m.shapes.size() != 4) return;
    const Shape2D& hull = m.shapes[0];
    CHECK(hull.points.size() == 3 && hull.closed);
    CHECK(hull.fill && hull.fill->r == 255 && hull.fill->g == 255);
    CHECK(hull.stroke && hull.stroke->r == 0 && AlmostEqual(hull.stroke_width, 0.5));
    CHECK(hull.part.empty());  // the root svg id is not a part

    const Shape2D& rect = m.shapes[1];
    CHECK(rect.fill && rect.fill->r == 0x10 && rect.fill->g == 0x20 && rect.fill->b == 0x30);
    CHECK(!rect.stroke);

    // Circle: inherited fill, translated by the group
    const Shape2D& circle = m.shapes[2];
    CHECK(circle.fill && circle.fill->r == 255 && circle.fill->g == 0);
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& p : circle.points) {
        cx += p.x / circle.points.size();
        cy += p.y / circle.points.size();
    }
    CHECK(AlmostEqual(cx, 1.0, 1e-6) && AlmostEqual(cy, 2.0, 1e-6));

    const Shape2D& line = m.shapes[3];
    CHECK(!line.closed && !line.fill && line.stroke && line.stroke->b == 255);

    CHECK(AlmostEqual(m.Length(), 20.0));
    CHECK(AlmostEqual(m.Breadth(), 10.0));
    CHECK(m.warnings.size() == 1);  // <text>
}

void TestSvgPath() {
    // Relative/absolute commands, implicit lineto after M, H/V, Z then
    // a second subpath, and a curve whose end point must be exact
    const Model2D m = ParseSvg(R"svg(<svg>
  <path d="M0,0 10,0 v5 H0 z m 20,0 l 5,0 c 0,5 -5,5 -5,0" fill="#000"/>
  <path d="M0 0 A5 5 0 0 1 10 0" stroke="#000" fill="none"/>
</svg>)svg",
                               "path.svg");
    CHECK(m.shapes.size() == 3);
    if (m.shapes.size() != 3) return;
    const auto& square = m.shapes[0];
    CHECK(square.closed && square.points.size() == 4);
    CHECK(AlmostEqual(square.points[2].x, 10.0) && AlmostEqual(square.points[3].x, 0.0) && AlmostEqual(square.points[3].y, 5.0));

    // m after z is relative to the closed subpath's start (0,0)
    const auto& curve = m.shapes[1];
    CHECK(!curve.closed);
    CHECK(AlmostEqual(curve.points.front().x, 20.0));
    CHECK(AlmostEqual(curve.points.back().x, 20.0) && AlmostEqual(curve.points.back().y, 0.0));

    // Half circle of radius 5 from (0,0) to (10,0), sweeping through y = -5
    // (sweep flag 1 is clockwise on screen, so through negative y)
    const auto& arc = m.shapes[2];
    double min_y = 0.0;
    for (const auto& p : arc.points) min_y = std::min(min_y, p.y);
    CHECK(AlmostEqual(min_y, -5.0, 0.05));
    CHECK(AlmostEqual(arc.points.back().x, 10.0) && AlmostEqual(arc.points.back().y, 0.0));
}

void TestSvgParts() {
    // Part names come from ids. Inside a moving part, child ids do not
    // rename it. The pivot is the leading edge unless data-pivot is set.
    const Model2D m = ParseSvg(R"svg(<svg>
  <g id="hull"><rect x="-10" y="-2" width="20" height="4" fill="#fff"/></g>
  <g id="rudder"><path id="path42" d="M -9,-0.5 H -12 V 0.5 H -9 Z" fill="red"/></g>
</svg>)svg",
                               "parts.svg");
    CHECK(m.shapes.size() == 2);
    if (m.shapes.size() != 2) return;
    CHECK(m.shapes[0].part == "hull");
    CHECK(m.shapes[1].part == "rudder");
    CHECK(AlmostEqual(m.Pivot("rudder").x, -9.0) && AlmostEqual(m.Pivot("rudder").y, 0.0));

    const Model2D explicit_pivot = ParseSvg(
        R"svg(<svg><rect id="rudder" data-pivot="-9.5,0.25" x="-12" y="-1" width="3" height="2" fill="red"/></svg>)svg", "p.svg");
    CHECK(AlmostEqual(explicit_pivot.Pivot("rudder").x, -9.5) && AlmostEqual(explicit_pivot.Pivot("rudder").y, 0.25));

    CHECK(FindPartMotion("rudder") != nullptr && FindPartMotion("rudder")->column == "delta_r");
    CHECK(FindPartMotion("hull") == nullptr);
}

void TestSvgErrors() {
    CHECK(SvgThrows(""));
    CHECK(SvgThrows("<rect/>"));                     // no <svg>
    CHECK(SvgThrows("<svg><rect width=\"1\""));      // unterminated tag
    CHECK(SvgThrows("<svg><!-- never closed"));
    // Recoverable problems are warnings
    const Model2D m = ParseSvg(R"svg(<svg><path d="M0,0 L1,1 L2" stroke="#000"/>
        <rect width="1" height="1" fill="chartreuse-ish"/></svg>)svg", "w.svg");
    CHECK(m.warnings.size() == 2);
    CHECK(m.shapes.size() == 2);
}

void TestDefault2D() {
    const Model2D m = DefaultModel2D(70.0, 10.0);
    CHECK(!m.Empty());
    CHECK(AlmostEqual(m.Breadth(), 10.0));
    CHECK(m.Length() > 70.0);  // the rudder sticks out aft
    CHECK(AlmostEqual(m.Pivot("rudder").x, -35.0));
}

Materials TestMaterials(const std::string& name) {
    if (name != "test.mtl") throw std::runtime_error("no " + name);
    std::istringstream mtl("newmtl red\nKd 1 0 0\nnewmtl glass\nKd 0 0 1\nd 0.5\n");
    return ParseMtl(mtl);
}

void TestObj() {
    std::istringstream obj(R"svg(# a quad hull and a triangle rudder
mtllib test.mtl
mtllib missing.mtl
v 10 -2 0
v 10 2 0
v -10 2 0
v -10 -2 0
vn 0 0 -2
o hull
usemtl red
f 1//1 2//1 3//1 4//1
o rudder
usemtl glass
v -10 0 1
v -12 0 1
v -10 0 3
f -3/7 -2 -1
usemtl nope
)svg");
    const Model3D m = ParseObj(obj, "test.obj", TestMaterials);
    CHECK(m.parts.size() == 2);
    CHECK(m.Triangles() == 3);
    if (m.parts.size() != 2) return;

    const Part3D& hull = m.parts[0];
    CHECK(hull.name == "hull" && hull.Triangles() == 2);
    CHECK(hull.colors[0].r == 255 && hull.colors[0].b == 0);
    CHECK(AlmostEqual(hull.normals[0].z, -1.0));  // given normal, normalized

    const Part3D& rudder = m.parts[1];
    CHECK(rudder.name == "rudder" && rudder.Triangles() == 1);
    CHECK(rudder.colors[0].b == 255 && rudder.colors[0].a == 128);
    // Flat normal of a triangle in the x-z plane points along +-y
    CHECK(AlmostEqual(std::abs(rudder.normals[0].y), 1.0));
    CHECK(AlmostEqual(m.Pivot("rudder").x, -10.0) && AlmostEqual(m.Pivot("rudder").z, 2.0));

    CHECK(AlmostEqual(m.Length(), 22.0));
    CHECK(m.warnings.size() == 2);  // missing.mtl, unknown material

    std::istringstream bad("v 0 0 0\nf 1 2 3\n");
    bool threw = false;
    try {
        ParseObj(bad, "bad.obj", TestMaterials);
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("bad.obj:2") != std::string::npos;
    }
    CHECK(threw);
}

void TestDefault3D() {
    const Model3D m = DefaultModel3D(70.0, 10.0, 3.0);
    CHECK(m.warnings.empty());
    CHECK(m.parts.size() == 2);
    CHECK(m.Triangles() == 3 + 3 + 10 + 2);  // deck, keel, sides, rudder
    CHECK(AlmostEqual(m.bounds.Size().y, 10.0));
    CHECK(AlmostEqual(m.bounds.max.z, 3.0));
    CHECK(AlmostEqual(m.Pivot("rudder").x, -35.0));
}

// Every vessel in the repository must have models that load cleanly
void TestRepositoryVessels() {
    const auto names = ListVessels(PROJECT_ROOT);
    CHECK(!names.empty());
    for (const auto& name : names) {
        const auto dir = VesselDir(PROJECT_ROOT, name);
        try {
            const Model2D m2 = LoadModel2D(dir / kModel2dFile);
            CHECK(m2.warnings.empty());
            CHECK(m2.Length() > 1.0 && m2.Breadth() > 0.1);
            const Model3D m3 = LoadModel3D(dir / kModel3dFile);
            CHECK(m3.warnings.empty());
            CHECK(m3.Triangles() > 100);
            // Both views of one vessel should agree on its size
            CHECK(std::abs(m2.Length() - m3.Length()) < 0.05 * m3.Length());
            for (const auto& motion : kPartMotions) {
                const bool in_2d = m2.pivots.count(std::string(motion.part)) != 0;
                const bool in_3d = m3.pivots.count(std::string(motion.part)) != 0;
                CHECK(in_2d == in_3d);
            }
        } catch (const std::runtime_error& e) {
            std::fprintf(stderr, "vessel %s: %s\n", name.c_str(), e.what());
            CHECK(false);
        }
    }
}

}  // namespace

int main() {
    TestSvgShapes();
    TestSvgPath();
    TestSvgParts();
    TestSvgErrors();
    TestDefault2D();
    TestObj();
    TestDefault3D();
    TestRepositoryVessels();
    return testing::Summary();
}
