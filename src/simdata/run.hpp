#pragma once
// ============================================================================
// Simulation runs on disk.
//
// Every simulation gets its own folder in simdata/:
//
//   simdata/20261003_142501_simulator_v1/
//     metadata.json     what was simulated: simulator, vessel, dt, columns, ...
//     simulation.csv    one row per time step, header = column names
//
// Folder names start with the local start time, so sorting them by name puts
// them in time order. metadata.json is written last, when the run is
// complete. Files from before this layout (simdata/simulator_v1/*.csv) are
// still listed, as runs without metadata.
// ============================================================================
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

namespace simdata {

// Bump when metadata.json changes in a way old readers cannot handle
inline constexpr int kFormatVersion = 1;
inline constexpr std::string_view kMetadataFile = "metadata.json";
inline constexpr std::string_view kDataFile = "simulation.csv";

struct Column {
    std::string name;
    std::string unit;         // "" when dimensionless
    std::string description;
};

struct Metadata {
    int format_version{kFormatVersion};
    std::string simulator;    // app that made the run, e.g. "simulator_v1"
    std::string vessel;       // folder name in vessels/, e.g. "test_vessel"
    std::string created;      // local start time, ISO 8601
    double dt{};              // [s] time step
    double duration{};        // [s] last time - first time
    std::size_t samples{};    // csv data rows
    std::string data_file{kDataFile};
    std::vector<Column> columns;
    // Anything else a simulator wants to keep: settings, enum names, ...
    nlohmann::ordered_json parameters = nlohmann::ordered_json::object();

    const Column* FindColumn(std::string_view name) const;
};

nlohmann::ordered_json ToJson(const Metadata& metadata);
// Missing keys keep their defaults, unknown keys are ignored.
// Throws std::runtime_error on wrong types or a newer format_version.
Metadata MetadataFromJson(const nlohmann::json& json);

// Both throw std::runtime_error with a readable message
void WriteMetadata(const std::filesystem::path& run_dir, const Metadata& metadata);
Metadata ReadMetadata(const std::filesystem::path& run_dir);

// Local time as YYYY-MM-DDTHH:MM:SS+hhmm
std::string IsoTimestamp(std::time_t when);
// YYYYMMDD_HHMMSS_<simulator>
std::string RunName(std::string_view simulator, std::time_t when);
// Creates <simdata_dir>/RunName(...), with _2, _3, ... appended when it
// already exists. Throws std::runtime_error.
std::filesystem::path CreateRunDir(const std::filesystem::path& simdata_dir,
                                   std::string_view simulator, std::time_t when);

// ---------------------------------------------------------------------------
// Finding runs
// ---------------------------------------------------------------------------

struct Run {
    std::string name;                    // folder name (legacy: csv file name)
    std::filesystem::path dir;           // run folder (legacy: folder of the csv)
    std::filesystem::path data_file;     // the csv
    std::filesystem::path metadata_file; // empty when the run has none

    bool HasMetadata() const { return !metadata_file.empty(); }
};

// True if dir holds metadata.json or simulation.csv
bool IsRunDir(const std::filesystem::path& dir);

// Runs in simdata_dir, newest first: run folders, plus loose csv files in
// simdata_dir and its subfolders (the layout before run folders)
std::vector<Run> ListRuns(const std::filesystem::path& simdata_dir);

// The run a path refers to: a run folder, its metadata.json, or a csv file.
// The csv a metadata file names is used when it is readable. nullopt if the
// path is none of these.
std::optional<Run> ResolveRun(const std::filesystem::path& path);

// ---------------------------------------------------------------------------
// Writing a run
// ---------------------------------------------------------------------------

// Creates the run folder and streams rows into simulation.csv. Finish()
// writes metadata.json with samples and duration filled in.
//
//   simdata::RunWriter run(simdata_dir, metadata);   // metadata.columns = csv header
//   for (...) run.WriteRow({t, x, y, ...});           // first column is time
//   run.Finish();
class RunWriter {
public:
    // Throws std::runtime_error. metadata.created is set when empty.
    RunWriter(const std::filesystem::path& simdata_dir, Metadata metadata, int precision = 10);

    // Throws std::invalid_argument when the row does not match the columns
    void WriteRow(const std::vector<double>& row);
    // Throws std::runtime_error if the csv or metadata.json can not be written
    void Finish();

    const std::filesystem::path& Dir() const { return dir_; }
    const Metadata& GetMetadata() const { return metadata_; }

private:
    std::filesystem::path dir_;
    Metadata metadata_;
    std::ofstream csv_;
    std::optional<double> first_t_;
    double last_t_{};
    bool finished_{false};
};

}  // namespace simdata
