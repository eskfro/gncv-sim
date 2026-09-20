#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "common.hpp"
#include "vessel.hpp"

std::atomic<bool> running{true};

void print_every_ten_seconds(int n);
void write_to_csv(std::vector<common::VesselSnapshot>& snapshots);
void simulator_v1_plotter();

// Simulation params
static constexpr int kSimTimeSeconds = 240;
static constexpr double dt = 0.025;
static const int N = std::lround(kSimTimeSeconds / dt);

int main() {

    // define things
    int n = 0;
    std::vector<common::VesselSnapshot> snapshots;
    vessel::Vessel vessel{};
    std::cout << "Starting simulation\n";

    // Simulate
    while (n < N) {
        vessel.Step(dt);
        snapshots.push_back(vessel.Snapshot());
        n++;
        print_every_ten_seconds(n);
    }
    std::cout << "\nSimulation finished, writing to disk\n";

    // Store simulation
    write_to_csv(snapshots);
    std::cout << "Simulation successfully saved in simdata/simulator_v1/\n";

    // Plot the result
    simulator_v1_plotter();

    return 0;

}

// Generate .csv file to output snapshots elements into each column
// The name of the csv file should be on the format YYYYMMDD_HH:MM:SS_simulator_v1.csv
// stored in project-root/simdata/simulator_v1
void write_to_csv(std::vector<common::VesselSnapshot>& snapshots) {
    // Filename: YYYYMMDD_HH:MM:SS_simulator_v1.csv
    std::time_t now = std::time(nullptr);
    std::ostringstream name;
    name << std::put_time(std::localtime(&now), "%Y%m%d_%H:%M:%S") << "_simulator_v1.csv";

    std::filesystem::path dir = "simdata/simulator_v1";
    std::filesystem::create_directories(dir);

    std::ofstream file(dir / name.str());
    if (!file) {
        std::cerr << "Could not open " << dir / name.str() << "\n";
        return;
    }

    // Header
    // guidance_mode: 0 = HeadingHold, 1 = PositionHold, 2 = WaypointTracking
    // *_ref = allocator reference, *_cmd = command after lowpass, no suffix = actuator state
    // delta_r in rad, n_* in rpm
    file << "t,x,y,z,phi,theta,psi,u,v,w,p,q,r,"
         << "tau_X,tau_Y,tau_Z,tau_K,tau_M,tau_N,"
         << "guidance_mode,x_d,y_d,psi_d,u_d,"
         << "delta_r_ref,delta_r_cmd,delta_r,"
         << "n_mp_ref,n_mp_cmd,n_mp,"
         << "n_tt_ref,n_tt_cmd,n_tt\n";

    // One row per snapshot: time, eta (6), nu (6), tau (6), reference (5), actuators (9)
    file << std::setprecision(10);
    for (const auto& s : snapshots) {
        file << s.t;
        for (int i = 0; i < 12; i++) file << ',' << s.x(i);
        for (int i = 0; i < 6; i++) file << ',' << s.tau(i);

        // Reference (cast the enum: uint8_t would print as a character)
        file << ',' << static_cast<int>(s.reference.guidance_mode)
             << ',' << s.reference.eta_d(0)
             << ',' << s.reference.eta_d(1)
             << ',' << s.reference.psi_d
             << ',' << s.reference.u_d;

        // Actuators: rudder, main propulsor, tunnel thruster
        file << ',' << s.actuator_references.delta_r
             << ',' << s.actuator_commands.delta_r
             << ',' << s.actuator_states.delta_r
             << ',' << s.actuator_references.n_mp
             << ',' << s.actuator_commands.n_mp
             << ',' << s.actuator_states.n_mp
             << ',' << s.actuator_references.n_tt
             << ',' << s.actuator_commands.n_tt
             << ',' << s.actuator_states.n_tt;
        file << '\n';
    }

    std::cout << "Wrote to " << name.str() << std::endl; 
}

void print_every_ten_seconds(int n) {
    const int n10s = 10 * std::lround(1 / dt);
    if (n % n10s != 0) return;
    int time_seconds = 10 * n / n10s;
    std::cout << time_seconds << " ";
}

// Starts the app
void simulator_v1_plotter() {
    const std::filesystem::path script = "apps/simulator_v1_plotter/main.py";
    if (!std::filesystem::exists(script)) {
        std::cerr << "Could not find " << script << " (run from the project root)\n";
        return;
    }

    const std::string cmd = "python3 " + script.string();
    if (std::system(cmd.c_str()) != 0) {
        std::cerr << "Plotter exited with an error\n";
    }
}
