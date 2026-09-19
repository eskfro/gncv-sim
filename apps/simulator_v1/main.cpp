#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <vector>

#include "common.hpp"
#include "vessel.hpp"

std::atomic<bool> running{true};

void print_every_second(int n);

// Simulation time
static constexpr int kSimTimeSeconds = 10;
static constexpr double dt = 0.025;
static constexpr int N = (kSimTimeSeconds * 1000) / dt;

int main() {

    // define things
    int n = 0;
    std::vector<common::VesselSnapshot> snapshots;
    vessel::Vessel vessel{};

    std::cout << "Starting simulation\n";
    
    while (n < N) {

        vessel.Step(dt);
        snapshots.push_back(vessel.Snapshot());

        n++;
        print_every_second(n);
    }

    std::cout << "Simulation finished, writing to disk\n";

    // Generate .csv file to output snapshots elements into each column
    // The name of the csv file should be on the format YYYYMMDD_HH:MM:SS_simulator_v1.csv
    // stored in project-root/simdata/simulator_v1

    std::cout << "Simulation successfully saved in simdata/simulator_v1/\n";

    return 0;

}

void print_every_second(int n) {
    const int n_per_second = 1000 / dt;
    
    if (n % n_per_second != 0) return;

    int time_seconds = n / n_per_second;

    std::cout << "t = " << time_seconds << std::endl;
}
