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

// Libs
#include "core/common.hpp"
#include "core/vessel.hpp"

// App
#include "simulator_v1.hpp"
#include "simulator_v1.cpp"

std::atomic<bool> running{true};

int main() {
    int n = 0;
    std::vector<common::VesselSnapshot> snapshots;
    snapshots.reserve(N);

    vessel::Vessel vessel{};
    vessel.Init();
    snapshots.push_back(vessel.Snapshot());

    std::cout << "Starting simulation\n";
    while (n < N && running) {
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

    simulator_v1_plotter(run_dir);

    return 0;

}




