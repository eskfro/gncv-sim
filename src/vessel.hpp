#pragma once

#include "common.hpp"

#include <stdlib.h>
#include <string>

class VesselDimensions {
public:
    double Length() { return length_; }
    double Width() { return width_; }
private:
    double length_;
    double width_;
};

class Vessel {
public:
    Vessel() = default;

    void Step(double dt);

private:
    std::string_view name_{"MyVessel"};
    common::Eta eta_{};
    common::Nu nu_{};

};