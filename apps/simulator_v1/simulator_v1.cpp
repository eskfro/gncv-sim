#include "simulator_v1.hpp"
#include "simdata/paths.hpp"

// Store the simulation in its own folder, simdata/YYYYMMDD_HHMMSS_simulator_v1/:
//   metadata.json   simulator, vessel, dt, columns with units, ...
//   simulation.csv  one row per snapshot
// Returns the folder. Throws std::exception on failure.
std::filesystem::path write_run(const std::vector<common::VesselSnapshot>& snapshots) {
    simdata::Metadata metadata;
    metadata.simulator = "simulator_v1";
    metadata.vessel = kVesselName;
    metadata.dt = p.dt;
    metadata.columns = kColumns;
    metadata.parameters["sim_time"] = p.sim_time;
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

    row.push_back(static_cast<int>(s.desired.guidance_mode));
    row.push_back(s.desired.eta_d(0));
    row.push_back(s.desired.eta_d(1));
    row.push_back(s.desired.psi_d);
    row.push_back(s.desired.u_d);

    for (const double value : {s.actuator_references.delta_r, s.actuator_commands.delta_r, s.actuator_states.delta_r,
                               s.actuator_references.n_mp, s.actuator_commands.n_mp, s.actuator_states.n_mp,
                               s.actuator_references.n_tt, s.actuator_commands.n_tt, s.actuator_states.n_tt}) {
        row.push_back(value);
    }
    return row;
}

void print_every_ten_seconds(int n) {
    const int n10s = 10 * std::lround(1 / p.dt);
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