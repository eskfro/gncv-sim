#include <cmath>
#include <sstream>
#include <string>

#include "math3d.hpp"
#include "orbit_camera.hpp"
#include "scene.hpp"
#include "simdata/recording.hpp"
#include "testing/check.hpp"
#include "vessel_pose.hpp"

using namespace playback3d;
using testing::AlmostEqual;

namespace {

bool Near(V3 a, V3 b, double tol = 1e-9) {
    return AlmostEqual(a.x, b.x, tol) && AlmostEqual(a.y, b.y, tol) && AlmostEqual(a.z, b.z, tol);
}

void TestRotations() {
    // Heading 90 deg: the bow (body x) points east
    CHECK(Near(TransformPoint(RotZYX(0, 0, M_PI / 2), {1, 0, 0}), {0, 1, 0}));
    // Positive roll: starboard (body y) goes down (z > 0)
    const V3 stbd = TransformPoint(RotZYX(0.1, 0, 0), {0, 1, 0});
    CHECK(stbd.z > 0.0 && AlmostEqual(stbd.z, std::sin(0.1)));
    // Positive pitch: the bow goes up (z < 0)
    CHECK(TransformPoint(RotZYX(0, 0.1, 0), {1, 0, 0}).z < 0.0);

    // A positive rudder angle swings the trailing edge (aft of the pivot) to port
    const V3 trailing = TransformPoint(PartRotation({-33, 0, 2}, 0.2), {-37, 0, 2});
    CHECK(trailing.y < 0.0);
    CHECK(Near(TransformPoint(PartRotation({-33, 0, 2}, 0.2), {-33, 0, 5}), {-33, 0, 5}));  // on the axis

    const Mat4 a = Translate({1, 2, 3}) * Scale(2.0);
    CHECK(Near(TransformPoint(a, {1, 1, 1}), {3, 4, 5}));
}

void TestCamera() {
    OrbitCamera cam;
    cam.SetViewport(100, 50, 800, 600);
    cam.SetTarget({500, -200, 0});
    cam.SetYaw(0.0);  // looking north
    cam.SetPitch(0.5);
    cam.SetDistance(100);

    // The target is in the middle of the viewport
    const auto c = cam.Project(cam.Target());
    CHECK(c && AlmostEqual(c->x, 500.0, 1e-6) && AlmostEqual(c->y, 350.0, 1e-6));
    // Looking north: east is to the right, the far (north) side is up
    const auto east = cam.Project(cam.Target() + V3{0, 10, 0});
    const auto north = cam.Project(cam.Target() + V3{10, 0, 0});
    CHECK(east && east->x > c->x);
    CHECK(north && north->y < c->y);
    // Points behind the camera are not projected
    CHECK(!cam.Project(cam.Eye() - cam.Forward() * 10.0));

    // The eye is above the water, behind the target
    const V3 eye = cam.Eye();
    CHECK(eye.z < 0.0 && eye.x < cam.Target().x);
    CHECK(AlmostEqual(Length(cam.Target() - eye), 100.0, 1e-9));

    // Limits
    cam.SetPitch(-1.0);
    CHECK(AlmostEqual(cam.Pitch(), OrbitCamera::kMinPitch));
    cam.Zoom(1e9);
    CHECK(AlmostEqual(cam.Distance(), OrbitCamera::kMinDistance));

    // Pan: dragging right moves the world right, so the target moves west of east
    cam.SetDistance(100);
    const V3 before = cam.Target();
    cam.Pan(50, 0);
    CHECK(cam.Target().y < before.y && AlmostEqual(cam.Target().x, before.x, 1e-9));

    // Fit puts the box centre in view
    cam.Fit({0, 0, 0}, {1000, 200, 0});
    CHECK(Near(cam.Target(), {500, 100, 0}));
    const auto corner = cam.Project({1000, 200, 0});
    CHECK(corner && corner->x > 100 && corner->x < 900 && corner->y > 50 && corner->y < 650);
}

void TestScene() {
    SceneBatch scene;
    scene.Clear({1000, 2000, 0});
    scene.Line({1000, 2000, 0}, {1001, 2000, 0}, Rgb8(255, 0, 0));
    CHECK(scene.Lines().size() == 2);
    CHECK(AlmostEqual(scene.Lines()[1].x, 1.0) && AlmostEqual(scene.Lines()[1].r, 1.0));  // relative to the origin

    // Ribbon: two quads for three points, width honoured on a straight run
    scene.Ribbon({{1000, 2000, 0}, {1010, 2000, 0}, {1020, 2000, 0}}, 2.0, Rgb8(0, 0, 255));
    CHECK(scene.Triangles().size() == 12);
    double min_y = 1e9;
    double max_y = -1e9;
    for (const auto& v : scene.Triangles()) {
        min_y = std::min(min_y, static_cast<double>(v.y));
        max_y = std::max(max_y, static_cast<double>(v.y));
    }
    CHECK(AlmostEqual(max_y - min_y, 2.0, 1e-6));

    // A sharp turn is limited by the miter
    scene.Clear({});
    scene.Ribbon({{0, 0, 0}, {10, 0, 0}, {0, 0.01, 0}}, 2.0, Rgb8(0, 0, 255));
    for (const auto& v : scene.Triangles()) CHECK(std::abs(v.x) < 20.0 && std::abs(v.y) < 20.0);

    scene.Clear({});
    scene.Ribbon({{0, 0, 0}}, 2.0, Rgb8(0, 0, 255));  // too short: nothing
    scene.Arrow({0, 0, 0}, {0, 0, 0}, 1.0, Rgb8(0, 0, 0));
    CHECK(scene.Triangles().empty());
    scene.Disc({0, 0, 0}, 1.0, Rgb8(0, 0, 0), 8);
    CHECK(scene.Triangles().size() == 24);
    scene.Text({1, 2, 3}, "hello", Rgb8(0, 0, 0));
    CHECK(scene.Labels().size() == 1 && scene.Labels()[0].text == "hello");
    CHECK(!scene.GetVessel());
    scene.Vessel({Mat4{}, {}});
    CHECK(scene.GetVessel().has_value());
}

void TestPose() {
    std::istringstream csv("t,x,y,z,phi,theta,psi\n0,10,20,0.5,0.1,0.02,1.0\n1,10,20,0.5,0.1,0.02,1.0\n");
    const auto rec = simdata::Recording::ParseCsv(csv, "pose.csv");
    const auto f = rec.Sample(0.5);
    const VesselPose pose = PoseAt(f, {3.0, 2.0});
    CHECK(Near(pose.position, {10, 20, 1.0}));
    CHECK(AlmostEqual(pose.phi, 0.3) && AlmostEqual(pose.theta, 0.06) && AlmostEqual(pose.psi, 1.0));

    // The body origin lands at the position relative to the origin
    const Mat4 m = BodyToWorld(pose, {10, 0, 0}, 2.0);
    CHECK(Near(TransformPoint(m, {0, 0, 0}), {0, 20, 1.0}));
    // The size scale applies to body points
    CHECK(AlmostEqual(Length(TransformPoint(m, {1, 0, 0}) - TransformPoint(m, {0, 0, 0})), 2.0));

    // Older files without z, phi, theta: flat on the water
    std::istringstream old_csv("t,x,y,psi\n0,1,2,0.5\n");
    const auto old_rec = simdata::Recording::ParseCsv(old_csv, "old.csv");
    const VesselPose flat = PoseAt(old_rec.Sample(0.0));
    CHECK(Near(flat.position, {1, 2, 0}) && flat.phi == 0.0 && flat.theta == 0.0);
}

}  // namespace

int main() {
    TestRotations();
    TestCamera();
    TestScene();
    TestPose();
    return testing::Summary();
}
