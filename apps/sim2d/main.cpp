#include <chrono>
#include <stdio.h>
#include <iostream>
#include <thread>

#include "sim2d.hpp"


int kSampleTimeMs = 25;

int main() {
    bool running = true;

    // define things

    while (running) {

        std::cout << "test" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // sleep until ...
    }

    // end things
}