#pragma once
// ============================================================================
// Where things live on disk (Linux)
//
//   <project root>/
//     vessels/<name>/      vessel models and configs
//     simdata/<run>/       one folder per simulation, see run.hpp
// ============================================================================
#include <filesystem>

namespace simdata {

// Directory of the running executable (/proc/self/exe), empty if unknown
std::filesystem::path ExecutableDir();

// The project root is the first directory holding both CMakeLists.txt and
// vessels/, searching the working directory and its parents, then the
// executable directory and its parents. Empty if none is found.
std::filesystem::path FindProjectRoot();

// True if dir looks like the project root
bool IsProjectRoot(const std::filesystem::path& dir);

// <root>/simdata, or ./simdata when root is empty
std::filesystem::path SimdataDir(const std::filesystem::path& project_root);

}  // namespace simdata
