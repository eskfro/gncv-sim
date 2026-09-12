// #pragma twice
#pragma once

#include "common.hpp"

namespace guidance {

/*
Guides according to the mission
*/
class Guidance {
public:     
    void Step(double dt);

    common::Reference Reference() { return reference_; }

private:
    double time_;
    common::GuidanceMode guidance_mode_;
    common::Reference reference_;
};

} // namespace control