#pragma once
// ============================================================================
// The vessels/ folder in the project root. Each vessel has its own folder:
//
//   vessels/<name>/
//     model_2d.svg    top view for 2D tools, see model_2d.hpp
//     model_3d.obj    3D mesh (+ model_3d.mtl colors), see model_3d.hpp
//
// A run's metadata.json names its vessel, so tools find the models from it.
// Physics configs will live next to the models later.
// ============================================================================
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace vessel_model {

inline constexpr std::string_view kVesselsDir = "vessels";
inline constexpr std::string_view kModel2dFile = "model_2d.svg";
inline constexpr std::string_view kModel3dFile = "model_3d.obj";

// Used when a run does not say which vessel it is (runs from before metadata)
inline constexpr std::string_view kDefaultVessel = "test_vessel";

// <project_root>/vessels/<name>. Does not check that it exists.
std::filesystem::path VesselDir(const std::filesystem::path& project_root, std::string_view name);

// Names of the folders in <project_root>/vessels, sorted
std::vector<std::string> ListVessels(const std::filesystem::path& project_root);

}  // namespace vessel_model
