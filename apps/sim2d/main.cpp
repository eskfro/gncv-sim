#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdio.h>
#include <iostream>
#include <thread>

#include "sim2d.hpp"


std::atomic<bool> running{true};

int kSampleTimeMs = 25;

int main() {

    // define things
    uint64_t count = 1;

    while (running) {

        std::cout << "test - " << count << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        count++;
        // sleep until ...
    }

    // end things
}