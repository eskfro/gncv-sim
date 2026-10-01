#pragma once
// ============================================================================
// Knowledge about the simulator_v1 csv format, kept in one place.
//
// Everything else in playback-2d looks columns up by name, so supporting a new
// column (or a new simulator) means editing this file and adding a layer or
// panel that uses it.
// ============================================================================
#include <array>
#include <string_view>
#include <vector>

namespace playback2d {

// How a column is interpolated between two rows
enum class Interp {
    Linear,  // plain linear interpolation
    Angle,   // linear along the shortest arc, result wrapped to (-pi, pi]
    Hold,    // value of the earlier row (discrete signals such as modes)
};

namespace col {
// Time and state: eta = (x, y, z, phi, theta, psi), nu = (u, v, w, p, q, r)
inline constexpr std::string_view kT = "t";
inline constexpr std::string_view kX = "x";
inline constexpr std::string_view kY = "y";
inline constexpr std::string_view kPhi = "phi";
inline constexpr std::string_view kTheta = "theta";
inline constexpr std::string_view kPsi = "psi";
inline constexpr std::string_view kU = "u";
inline constexpr std::string_view kV = "v";
inline constexpr std::string_view kR = "r";

// Generalized forces in body frame
inline constexpr std::string_view kTauX = "tau_X";
inline constexpr std::string_view kTauY = "tau_Y";
inline constexpr std::string_view kTauN = "tau_N";

// Reference (added after the first csv version)
inline constexpr std::string_view kGuidanceMode = "guidance_mode";
inline constexpr std::string_view kXd = "x_d";
inline constexpr std::string_view kYd = "y_d";
inline constexpr std::string_view kPsiD = "psi_d";
inline constexpr std::string_view kUd = "u_d";

// Actuator states. delta_r in rad, n_* in rpm
inline constexpr std::string_view kDeltaR = "delta_r";
inline constexpr std::string_view kNmp = "n_mp";
inline constexpr std::string_view kNtt = "n_tt";
}  // namespace col

// Columns without which there is nothing to play back
inline const std::vector<std::string_view> kRequiredColumns = {col::kT, col::kX, col::kY,
                                                               col::kPsi};

// Same order as common::GuidanceMode in src/common.hpp
inline constexpr std::array<const char*, 3> kGuidanceModes = {"HeadingHold", "PositionHold",
                                                              "WaypointTracking"};

inline Interp DefaultInterp(std::string_view column) {
    if (column == col::kGuidanceMode) return Interp::Hold;
    if (column == col::kPhi || column == col::kTheta || column == col::kPsi ||
        column == col::kPsiD) {
        return Interp::Angle;
    }
    return Interp::Linear;
}

// Hull size used for drawing. Matches vessel::VesselParams in src/dynamics.hpp,
// which is not stored in the csv.
struct ShipGeometry {
    double length = 70.0;   // [m]
    double breadth = 10.0;  // [m]
};

}  // namespace playback2d
