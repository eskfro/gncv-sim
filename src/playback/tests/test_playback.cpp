#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "playback/playback_clock.hpp"
#include "playback/session.hpp"
#include "testing/check.hpp"

using namespace playback;
using testing::AlmostEqual;
namespace fs = std::filesystem;

namespace {

struct TempDir {
    fs::path path;
    TempDir() {
        char pattern[] = "/tmp/test_playback_XXXXXX";
        const char* made = mkdtemp(pattern);
        if (made == nullptr) throw std::runtime_error("mkdtemp failed");
        path = made;
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

void WriteFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path) << text;
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

void TestSession() {
    TempDir tmp;
    const fs::path root = tmp.path;
    const fs::path simdata = root / "simdata";
    fs::create_directories(root / "vessels" / "test_vessel");
    fs::create_directories(root / "vessels" / "other_vessel");
    const std::string csv = "t,x,y,psi,guidance_mode\n0,0,0,0,1\n2,4,2,0.5,1\n";
    WriteFile(simdata / "20260101_100000_sim" / "simulation.csv", csv);
    WriteFile(simdata / "20260101_100000_sim" / "metadata.json",
              R"({"simulator": "sim", "vessel": "other_vessel", "parameters": {"guidance_modes": ["A", "B"]}})");
    WriteFile(simdata / "old" / "20250101_10:00:00_simulator_v1.csv", csv);
    WriteFile(simdata / "20260101_110000_sim" / "simulation.csv", "t,x\n0,1\n");  // missing y, psi
    WriteFile(simdata / "20260101_120000_sim" / "metadata.json", R"({"vessel": "ghost"})");  // no csv

    Session s;
    s.SetProjectRoot(root);
    s.SetSimdataDir(simdata);
    s.Refresh();
    CHECK(s.Runs().size() == 4);
    CHECK(!s.Loaded());
    CHECK(s.LoadId() == 0);

    // New run: vessel from metadata, clock rewound and playing
    CHECK(s.Open(simdata / "20260101_100000_sim"));
    CHECK(s.Loaded() && s.LoadId() == 1);
    CHECK(s.GetMetadata() != nullptr && s.GetMetadata()->simulator == "sim");
    CHECK(s.VesselName() == "other_vessel");
    CHECK(s.VesselDir() == root / "vessels" / "other_vessel");
    CHECK(s.Notes().empty());
    CHECK(s.GuidanceModes().size() == 2 && s.GuidanceModes()[1] == "B");
    CHECK(AlmostEqual(s.clock.End(), 2.0) && s.clock.Playing());
    CHECK(s.IsCurrent(s.Runs()[2]));  // newest first: 120000, 110000, 100000, legacy

    // Failures keep the loaded run
    CHECK(!s.Open(simdata / "20260101_110000_sim"));
    CHECK(s.Error().find("missing columns") != std::string::npos);
    CHECK(!s.Open(simdata / "20260101_120000_sim"));
    CHECK(!s.Open(root / "nothing_here"));
    CHECK(s.LoadId() == 1 && s.VesselName() == "other_vessel");

    // Legacy csv: no metadata, default vessel, default guidance modes
    CHECK(s.Open(simdata / "old" / "20250101_10:00:00_simulator_v1.csv"));
    CHECK(s.Error().empty());
    CHECK(s.GetMetadata() == nullptr);
    CHECK(s.VesselName() == "test_vessel" && !s.VesselDir().empty());
    CHECK(s.Notes().size() == 2);  // no metadata, assumed vessel
    CHECK(s.GuidanceModes().size() == 3);

    // Override wins over metadata, unknown vessels are reported
    s.SetVesselOverride("ghost");
    CHECK(s.Open(simdata / "20260101_100000_sim" / "metadata.json"));
    CHECK(s.VesselName() == "ghost" && s.VesselDir().empty());
    CHECK(s.Notes().size() == 1);
}

}  // namespace

int main() {
    TestClock();
    TestSession();
    return testing::Summary();
}
