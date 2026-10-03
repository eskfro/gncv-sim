#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <vector>

#include "common.hpp"
#include "vessel.hpp"
#include "simdata/paths.hpp"
#include "simdata/run.hpp"

std::atomic<bool> running{true};

// Declare functions
void print_every_ten_seconds(int n);
std::filesystem::path write_run(const std::vector<common::VesselSnapshot>& snapshots);
std::vector<double> snapshot_row(const common::VesselSnapshot& s);
void simulator_v1_plotter(const std::filesystem::path& run_dir);

// Simulation params
static constexpr int kSimTimeSeconds = 240;
static constexpr double dt = 0.025;
static const int N = std::lround(kSimTimeSeconds / dt);

// Folder in vessels/ with the models of the simulated vessel. Its size
// matches vessel::VesselParams in src/dynamics.hpp.
static constexpr const char* kVesselName = "test_vessel";

// Same order as common::GuidanceMode in src/common.hpp
static const std::vector<std::string> kGuidanceModes = {"HeadingHold", "PositionHold", "WaypointTracking"};

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

int main() {

    // Init
    int n = 0;
    std::vector<common::VesselSnapshot> snapshots;
    snapshots.reserve(N);

    vessel::Vessel vessel{};
    vessel.Init();
    snapshots.push_back(vessel.Snapshot());

    std::cout << "Starting simulation\n";
    while (n < N) {
        vessel.Step(dt);
        snapshots.push_back(vessel.Snapshot());
        n++;
        print_every_ten_seconds(n);
    }
    std::cout << "\nSimulation finished, writing to disk\n";

    // Store simulation
    std::filesystem::path run_dir;
    try {
        run_dir = write_run(snapshots);
    } catch (const std::exception& e) {
        std::cerr << "Could not save the simulation: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Simulation successfully saved in " << run_dir.string() << "/\n";

    // Plot the result
    simulator_v1_plotter(run_dir);

    return 0;

}

// Store the simulation in its own folder, simdata/YYYYMMDD_HHMMSS_simulator_v1/:
//   metadata.json   simulator, vessel, dt, columns with units, ...
//   simulation.csv  one row per snapshot
// Returns the folder. Throws std::exception on failure.
std::filesystem::path write_run(const std::vector<common::VesselSnapshot>& snapshots) {
    simdata::Metadata metadata;
    metadata.simulator = "simulator_v1";
    metadata.vessel = kVesselName;
    metadata.dt = dt;
    metadata.columns = kColumns;
    metadata.parameters["sim_time"] = kSimTimeSeconds;
    metadata.parameters["guidance_modes"] = kGuidanceModes;

    const std::filesystem::path simdata_dir = simdata::SimdataDir(simdata::FindProjectRoot());
    simdata::RunWriter run(simdata_dir, metadata);
    for (const auto& s : snapshots) run.WriteRow(snapshot_row(s));
    run.Finish();
    return run.Dir();
}

// One csv row: time, eta (6), nu (6), tau (6), reference (5), actuators (9)
std::vector<double> snapshot_row(const common::VesselSnapshot& s) {
    std::vector<double> row;
    row.reserve(kColumns.size());
    row.push_back(s.t);
    for (int i = 0; i < 12; i++) row.push_back(s.x(i));
    for (int i = 0; i < 6; i++) row.push_back(s.tau(i));

    row.push_back(static_cast<int>(s.reference.guidance_mode));
    row.push_back(s.reference.eta_d(0));
    row.push_back(s.reference.eta_d(1));
    row.push_back(s.reference.psi_d);
    row.push_back(s.reference.u_d);

    for (const double value : {s.actuator_references.delta_r, s.actuator_commands.delta_r, s.actuator_states.delta_r,
                               s.actuator_references.n_mp, s.actuator_commands.n_mp, s.actuator_states.n_mp,
                               s.actuator_references.n_tt, s.actuator_commands.n_tt, s.actuator_states.n_tt}) {
        row.push_back(value);
    }
    return row;
}

void print_every_ten_seconds(int n) {
    const int n10s = 10 * std::lround(1 / dt);
    if (n % n10s != 0) return;
    int time_seconds = 10 * n / n10s;
    std::cout << time_seconds << " ";
}

// Starts the app with the new run selected
void simulator_v1_plotter(const std::filesystem::path& run_dir) {
    const std::filesystem::path script = simdata::FindProjectRoot() / "apps/simulator_v1_plotter/main.py";
    if (!std::filesystem::exists(script)) {
        std::cerr << "Could not find " << script << " (run from the project root)\n";
        return;
    }

    const std::string cmd = "python3 \"" + script.string() + "\" \"" + run_dir.string() + "\"";
    if (std::system(cmd.c_str()) != 0) {
        std::cerr << "Plotter exited with an error\n";
    }
}
