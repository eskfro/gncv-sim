#pragma once

#include "simdata/run.hpp"
#include "core/common.hpp"
#include "core/config.hpp"

// Simulation params
static constexpr config::SimParams p{};

static const int N = std::lround(p.sim_time / p.dt);

// Folder in vessels/ with the models of the simulated vessel. Its size
// matches vessel::VesselParams in src/dynamics.hpp.
static constexpr const char* kVesselName = "test_vessel";

// Same order as common::GuidanceMode in src/common.hpp
static const std::vector<std::string> kGuidanceModes = {"HeadingHold", "PositionHold", "WaypointTracking"};

// Declare functions
void print_every_ten_seconds(int n);
std::filesystem::path write_run(const std::vector<common::VesselSnapshot>& snapshots);
std::vector<double> snapshot_row(const common::VesselSnapshot& s);
void simulator_v1_plotter(const std::filesystem::path& run_dir);

// Csv columns, in the order snapshot_row() writes them
// *_ref = allocator reference, *_cmd = command after lowpass, no suffix = actuator state
static const std::vector<simdata::Column> kColumns = {
    {"t", "s", "simulation time"},
    // eta, NED frame
    {"x", "m", "north position"},
    {"y", "m", "east position"},
    {"z", "m", "down position"},
    {"phi", "rad", "roll angle"},
    {"theta", "rad", "pitch angle"},
    {"psi", "rad", "yaw angle (heading)"},
    // nu, body frame
    {"u", "m/s", "surge velocity"},
    {"v", "m/s", "sway velocity"},
    {"w", "m/s", "heave velocity"},
    {"p", "rad/s", "roll rate"},
    {"q", "rad/s", "pitch rate"},
    {"r", "rad/s", "yaw rate"},
    // tau, body frame
    {"tau_X", "N", "surge force"},
    {"tau_Y", "N", "sway force"},
    {"tau_Z", "N", "heave force"},
    {"tau_K", "Nm", "roll moment"},
    {"tau_M", "Nm", "pitch moment"},
    {"tau_N", "Nm", "yaw moment"},
    // Reference
    {"guidance_mode", "", "index into parameters.guidance_modes"},
    {"x_d", "m", "desired north position"},
    {"y_d", "m", "desired east position"},
    {"psi_d", "rad", "desired heading"},
    {"u_d", "m/s", "desired surge velocity"},
    // Actuators: rudder, main propulsor, tunnel thruster
    {"delta_r_ref", "rad", "rudder angle, allocator reference"},
    {"delta_r_cmd", "rad", "rudder angle, command after lowpass"},
    {"delta_r", "rad", "rudder angle"},
    {"n_mp_ref", "rpm", "main propulsor, allocator reference"},
    {"n_mp_cmd", "rpm", "main propulsor, command after lowpass"},
    {"n_mp", "rpm", "main propulsor speed"},
    {"n_tt_ref", "rpm", "tunnel thruster, allocator reference"},
    {"n_tt_cmd", "rpm", "tunnel thruster, command after lowpass"},
    {"n_tt", "rpm", "tunnel thruster speed"},
};