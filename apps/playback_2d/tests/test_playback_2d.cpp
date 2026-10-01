#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>

#include "camera.hpp"
#include "playback_clock.hpp"
#include "recording.hpp"

using namespace playback2d;

namespace {

int failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                   \
        }                                                                 \
    } while (0)

bool AlmostEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

Recording Parse(const std::string& csv) {
    std::istringstream in(csv);
    return Recording::ParseCsv(in, "test.csv");
}

bool ParseThrows(const std::string& csv) {
    try {
        Parse(csv);
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}

void TestRecording() {
    // Same header layout as simulator_v1, trimmed. Includes a bad row, a row
    // going back in time and Windows line endings, which are all tolerated.
    const Recording rec = Parse(
        "t,x,y,psi,guidance_mode\r\n"
        "0.0,0,0,3.0,0\r\n"
        "1.0,10,-4,-3.0,0\r\n"
        "garbage\r\n"
        "0.5,99,99,0,0\r\n"
        "\r\n"
        "2.0,20,-8,-2.0,2\r\n");

    CHECK(rec.Rows() == 3);
    CHECK(rec.SkippedRows() == 2);
    CHECK(AlmostEqual(rec.StartTime(), 0.0));
    CHECK(AlmostEqual(rec.EndTime(), 2.0));
    CHECK(rec.Has("psi"));
    CHECK(!rec.Has("u"));
    CHECK(rec.MissingRequired().empty());

    CHECK(rec.IndexAt(-1.0) == 0);
    CHECK(rec.IndexAt(0.0) == 0);
    CHECK(rec.IndexAt(1.5) == 1);
    CHECK(rec.IndexAt(9.0) == 2);

    // Linear columns
    const Frame f = rec.Sample(0.25);
    CHECK(AlmostEqual(f.Time(), 0.25));
    CHECK(AlmostEqual(f.Get("x"), 2.5));
    CHECK(AlmostEqual(f.Get("y"), -1.0));
    CHECK(AlmostEqual(f.Get("missing", 7.0), 7.0));

    // psi goes 3.0 -> -3.0 the short way, across +-pi, not through zero
    const double psi = f.Get("psi");
    CHECK(std::abs(psi) > 3.0);
    const double step = 2.0 * M_PI - 6.0;  // short arc length
    CHECK(AlmostEqual(WrapAngle(psi - 3.0), 0.25 * step));

    // Discrete columns hold the earlier value
    CHECK(AlmostEqual(rec.Sample(1.9).Get("guidance_mode"), 0.0));
    CHECK(AlmostEqual(rec.Sample(2.0).Get("guidance_mode"), 2.0));

    // Clamped outside the recording
    CHECK(AlmostEqual(rec.Sample(-5.0).Get("x"), 0.0));
    CHECK(AlmostEqual(rec.Sample(50.0).Get("x"), 20.0));
    CHECK(AlmostEqual(rec.Sample(50.0).Time(), 2.0));

    // Errors
    CHECK(ParseThrows(""));
    CHECK(ParseThrows("x,y\n1,2\n"));          // no time column
    CHECK(ParseThrows("t,x,x\n1,2,3\n"));      // duplicate column
    CHECK(ParseThrows("t,x\n"));               // no rows
    CHECK(Parse("t,x\n0,1\n").MissingRequired().size() == 2);  // y, psi
}

void TestClock() {
    PlaybackClock clock;
    clock.Reset(10.0, 20.0);
    CHECK(AlmostEqual(clock.Time(), 10.0));
    CHECK(AlmostEqual(clock.Speed(), 1.0));  // real time by default
    CHECK(!clock.Playing());

    clock.Update(1.0);  // paused: nothing happens
    CHECK(AlmostEqual(clock.Time(), 10.0));

    clock.Play();
    clock.Update(1.0);
    CHECK(AlmostEqual(clock.Time(), 11.0));
    clock.SetSpeed(4.0);
    clock.Update(0.5);
    CHECK(AlmostEqual(clock.Time(), 13.0));
    CHECK(AlmostEqual(clock.Progress(), 0.3));

    // Stops at the end without loop, and Play() restarts
    clock.Update(100.0);
    CHECK(AlmostEqual(clock.Time(), 20.0));
    CHECK(!clock.Playing());
    clock.Play();
    CHECK(AlmostEqual(clock.Time(), 10.0));

    // Wraps with loop
    clock.SetLoop(true);
    clock.SetSpeed(1.0);
    clock.Seek(19.0);
    clock.Update(3.0);
    CHECK(AlmostEqual(clock.Time(), 12.0));
    CHECK(clock.Playing());

    clock.Seek(-100.0);
    CHECK(AlmostEqual(clock.Time(), 10.0));

    // Speed presets and limits
    clock.SetSpeed(1.0);
    clock.Faster();
    CHECK(AlmostEqual(clock.Speed(), 2.0));
    clock.SetSpeed(3.0);
    clock.Slower();
    CHECK(AlmostEqual(clock.Speed(), 2.0));
    clock.SetSpeed(1e9);
    CHECK(AlmostEqual(clock.Speed(), PlaybackClock::kMaxSpeed));
    clock.Faster();
    CHECK(AlmostEqual(clock.Speed(), PlaybackClock::kMaxSpeed));
    clock.SetSpeed(0.0);
    CHECK(AlmostEqual(clock.Speed(), PlaybackClock::kMinSpeed));
    clock.Slower();
    CHECK(AlmostEqual(clock.Speed(), PlaybackClock::kMinSpeed));
}

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
    TestRecording();
    TestClock();
    TestCamera();
    if (failures > 0) std::fprintf(stderr, "%d check(s) failed\n", failures);
    return failures == 0 ? 0 : 1;
}
