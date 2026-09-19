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

void print_every_second(int n);
void write_to_csv(std::vector<common::VesselSnapshot>& snapshots);

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
        print_every_second(n);
    }
    std::cout << "\nSimulation finished, writing to disk\n";

    // Store simulation
    write_to_csv(snapshots);
    std::cout << "Simulation successfully saved in simdata/simulator_v1/\n";
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
    file << "t,x,y,z,phi,theta,psi,u,v,w,p,q,r,"
         << "tau_X,tau_Y,tau_Z,tau_K,tau_M,tau_N\n";

    // One row per snapshot: time, eta (6), nu (6), tau (6)
    file << std::setprecision(10);
    for (const auto& s : snapshots) {
        file << s.t;
        for (int i = 0; i < 12; i++) file << ',' << s.x(i);
        for (int i = 0; i < 6; i++) file << ',' << s.tau(i);
        file << '\n';
    }

    std::cout << "Wrote to " << name.str() << std::endl; 
}

void print_every_second(int n) {
    const int n_per_second = std::lround(1 / dt);
    if (n % n_per_second != 0) return;
    int time_seconds = n / n_per_second;
    std::cout << time_seconds << " ";
}
