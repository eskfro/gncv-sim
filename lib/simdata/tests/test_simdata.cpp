#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "simdata/paths.hpp"
#include "simdata/recording.hpp"
#include "simdata/run.hpp"
#include "testing/check.hpp"

using namespace simdata;
using testing::AlmostEqual;
namespace fs = std::filesystem;

namespace {

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

// Fresh empty directory under the system temp dir, removed by the destructor
struct TempDir {
    fs::path path;
    TempDir() {
        char pattern[] = "/tmp/test_simdata_XXXXXX";
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

Metadata TestMetadata() {
    Metadata m;
    m.simulator = "simulator_test";
    m.vessel = "test_vessel";
    m.dt = 0.5;
    m.columns = {{"t", "s", "time"}, {"x", "m", "north"}, {"y", "m", "east"}, {"psi", "rad", "yaw"}};
    m.parameters["guidance_modes"] = {"A", "B"};
    return m;
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

void TestMetadataJson() {
    const Metadata m = TestMetadata();
    const Metadata back = MetadataFromJson(nlohmann::json::parse(ToJson(m).dump()));
    CHECK(back.simulator == "simulator_test");
    CHECK(back.vessel == "test_vessel");
    CHECK(AlmostEqual(back.dt, 0.5));
    CHECK(back.columns.size() == 4);
    CHECK(back.FindColumn("x") != nullptr && back.FindColumn("x")->unit == "m");
    CHECK(back.FindColumn("nope") == nullptr);
    CHECK(back.parameters["guidance_modes"][1] == "B");

    // Missing keys keep defaults, unknown keys are ignored
    const Metadata sparse = MetadataFromJson(nlohmann::json::parse(R"({"vessel": "v", "future_key": 1})"));
    CHECK(sparse.vessel == "v");
    CHECK(sparse.data_file == kDataFile);
    CHECK(sparse.format_version == kFormatVersion);

    bool threw = false;
    try {
        MetadataFromJson(nlohmann::json::parse(R"({"format_version": 999})"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        MetadataFromJson(nlohmann::json::parse(R"({"dt": "fast"})"));  // wrong type
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);
}

void TestRunWriter() {
    TempDir tmp;
    const fs::path simdata = tmp.path / "simdata";

    fs::path dir;
    {
        RunWriter run(simdata, TestMetadata());
        dir = run.Dir();
        CHECK(dir.parent_path() == simdata);
        CHECK(dir.filename().string().size() == std::string("YYYYMMDD_HHMMSS_simulator_test").size());
        CHECK(!fs::exists(dir / kMetadataFile));  // written by Finish()

        run.WriteRow({1.0, 0.0, 0.0, 0.0});
        run.WriteRow({1.5, 2.0, 3.0, 0.1});
        run.WriteRow({3.0, 4.0, 6.0, 0.2});
        bool threw = false;
        try {
            run.WriteRow({1.0, 2.0});
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
        run.Finish();
    }

    const Metadata m = ReadMetadata(dir);
    CHECK(m.samples == 3);
    CHECK(AlmostEqual(m.duration, 2.0));
    CHECK(!m.created.empty());
    CHECK(m.vessel == "test_vessel");

    const Recording rec = Recording::LoadCsv(dir / kDataFile);
    CHECK(rec.Rows() == 3);
    CHECK(AlmostEqual(rec.Sample(3.0).Get("y"), 6.0));

    // A second run in the same second gets a suffix instead of overwriting
    RunWriter second(simdata, TestMetadata());
    CHECK(second.Dir() != dir);
    second.Finish();
}

void TestListAndResolve() {
    TempDir tmp;
    const fs::path simdata = tmp.path / "simdata";
    const std::string csv = "t,x,y,psi\n0,0,0,0\n";

    // New layout, an unfinished run (no metadata yet), and legacy files
    WriteFile(simdata / "20260102_120000_simulator_v1" / "simulation.csv", csv);
    WriteFile(simdata / "20260102_120000_simulator_v1" / "metadata.json",
              R"({"simulator": "simulator_v1", "vessel": "test_vessel"})");
    WriteFile(simdata / "20260103_080000_simulator_v1" / "simulation.csv", csv);
    WriteFile(simdata / "simulator_v1" / "20260102_11:00:00_simulator_v1.csv", csv);
    WriteFile(simdata / "simulator_v1" / "20260102_13:00:00_simulator_v1.csv", csv);
    WriteFile(simdata / "simulator_v1" / "notes.txt", "not a run");
    WriteFile(simdata / "custom" / "metadata.json", R"({"data_file": "other.csv"})");
    WriteFile(simdata / "custom" / "other.csv", csv);

    const auto runs = ListRuns(simdata);
    CHECK(runs.size() == 5);
    if (runs.size() == 5) {
        // Newest first, legacy names sorted among the new ones
        CHECK(runs[0].name == "custom");  // not a timestamp: sorts after digits
        CHECK(runs[1].name == "20260103_080000_simulator_v1");
        CHECK(!runs[1].HasMetadata());
        CHECK(runs[2].name == "20260102_13:00:00_simulator_v1");
        CHECK(!runs[2].HasMetadata());
        CHECK(runs[3].name == "20260102_120000_simulator_v1");
        CHECK(runs[3].HasMetadata());
        CHECK(runs[4].name == "20260102_11:00:00_simulator_v1");
        CHECK(runs[0].data_file == simdata / "custom" / "other.csv");  // from metadata
    }
    CHECK(ListRuns(tmp.path / "missing").empty());

    // A run folder, its metadata.json or its csv all resolve to the run
    const fs::path run_dir = simdata / "20260102_120000_simulator_v1";
    for (const fs::path& p : {run_dir, run_dir / "metadata.json", run_dir / "simulation.csv"}) {
        const auto run = ResolveRun(p);
        CHECK(run && run->dir == run_dir && run->HasMetadata() && run->data_file == run_dir / "simulation.csv");
    }
    const auto legacy = ResolveRun(simdata / "simulator_v1" / "20260102_11:00:00_simulator_v1.csv");
    CHECK(legacy && !legacy->HasMetadata() && legacy->name == "20260102_11:00:00_simulator_v1");
    CHECK(!ResolveRun(simdata / "simulator_v1"));  // folder of legacy files, not a run
    CHECK(!ResolveRun(simdata / "simulator_v1" / "notes.txt"));
    CHECK(!ResolveRun(tmp.path / "missing.csv"));
}

void TestPaths() {
    TempDir tmp;
    CHECK(!IsProjectRoot(tmp.path));
    fs::create_directories(tmp.path / "vessels");
    WriteFile(tmp.path / "CMakeLists.txt", "");
    CHECK(IsProjectRoot(tmp.path));
    CHECK(SimdataDir(tmp.path) == tmp.path / "simdata");
    CHECK(SimdataDir({}) == fs::path("simdata"));
}

}  // namespace

int main() {
    TestRecording();
    TestMetadataJson();
    TestRunWriter();
    TestListAndResolve();
    TestPaths();
    return testing::Summary();
}
