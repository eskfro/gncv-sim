#pragma once
// ============================================================================
// Finding simulator_v1 csv files on disk (Linux)
// ============================================================================
#include <filesystem>
#include <vector>

namespace playback2d {

// Where simulator_v1 writes its csv files, relative to the project root
inline const std::filesystem::path kDefaultDataSubdir = "simdata/simulator_v1";

// Directory of the running executable (/proc/self/exe), empty if unknown
std::filesystem::path ExecutableDir();

// First existing <dir>/simdata/simulator_v1, trying the working directory and
// then the executable directory and its parents. Falls back to the working
// directory version even if it does not exist yet.
std::filesystem::path FindDataDir(const std::filesystem::path& exe_dir);

// *.csv files in dir, newest first. simulator_v1 names files by timestamp,
// so reverse name order is newest first (same as simulator_v1_plotter).
std::vector<std::filesystem::path> ListCsvFiles(const std::filesystem::path& dir);

}  // namespace playback2d
