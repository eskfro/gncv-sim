// Recording and PlaybackClock moved to lib/ and are tested there
#include "camera.hpp"
#include "testing/check.hpp"

using namespace playback2d;
using testing::AlmostEqual;

namespace {

void TestCamera() {
    Camera cam;
    cam.SetViewport(100.0, 50.0, 800.0, 600.0);
    cam.SetPixelsPerMeter(2.0);
    cam.CenterOn({0.0, 0.0});

    // Centre of the viewport, north is up, east is right
    const ScreenPoint c = cam.ToScreen({0.0, 0.0});
    CHECK(AlmostEqual(c.x, 500.0) && AlmostEqual(c.y, 350.0));
    const ScreenPoint n = cam.ToScreen({10.0, 0.0});
    CHECK(AlmostEqual(n.x, 500.0) && AlmostEqual(n.y, 330.0));
    const ScreenPoint e = cam.ToScreen({0.0, 10.0});
    CHECK(AlmostEqual(e.x, 520.0) && AlmostEqual(e.y, 350.0));

    // Round trip
    const NedPoint w = cam.ToWorld(cam.ToScreen({123.0, -45.0}));
    CHECK(AlmostEqual(w.north, 123.0) && AlmostEqual(w.east, -45.0));

    // Zoom keeps the anchor fixed
    const ScreenPoint anchor{700.0, 200.0};
    const NedPoint under = cam.ToWorld(anchor);
    cam.ZoomAt(3.0, anchor);
    CHECK(AlmostEqual(cam.PixelsPerMeter(), 6.0));
    const ScreenPoint after = cam.ToScreen(under);
    CHECK(AlmostEqual(after.x, anchor.x) && AlmostEqual(after.y, anchor.y));

    // Dragging the mouse right moves the world right
    const ScreenPoint before_pan = cam.ToScreen({0.0, 0.0});
    cam.Pan(30.0, -20.0);
    const ScreenPoint after_pan = cam.ToScreen({0.0, 0.0});
    CHECK(AlmostEqual(after_pan.x - before_pan.x, 30.0));
    CHECK(AlmostEqual(after_pan.y - before_pan.y, -20.0));

    // Fit: the limiting axis fills the viewport minus margins
    cam.Fit({0.0, 0.0}, {100.0, 400.0}, 0.0);
    CHECK(AlmostEqual(cam.PixelsPerMeter(), 2.0));  // 800 px / 400 m east
    CHECK(AlmostEqual(cam.Center().north, 50.0) && AlmostEqual(cam.Center().east, 200.0));
}

}  // namespace

int main() {
    TestCamera();
    return testing::Summary();
}
