#include "guidance.hpp"
#include "common.hpp"
#include <cmath>

namespace guidance {

void Guidance::Step(double dt) {
    const double t = time_;
    
    switch (guidance_mode_) {

    case common::GuidanceMode::HeadingHold: {

        reference_.guidance_mode = common::GuidanceMode::HeadingHold;

        const double psi_d = 30.0;
        const double sine = 60 * std::sin(t * 2 * M_PI / 80);
    
        // Heading reference generator
        if (common::inrange(t, 0, 60)) {
            reference_.psi_d = common::deg2rad(psi_d);
        } else if (common::inrange(t, 0, 0)) {
            reference_.psi_d = -common::deg2rad(psi_d);
        } else if (common::inrange(t, 60, 240)) {
            reference_.psi_d = common::deg2rad(sine);
        } else {
            reference_.psi_d = common::deg2rad(-180);
        }

        // Speed reference
        reference_.u_d = 14 * common::kKnots2Ms;
        reference_.eta_d.zeros(); // we dont use this in HeadingHold mode

        // Integrate time
        time_ += dt;

        break;
    }
    default:
        std::cout << "This should not be printed\n";
    }   
}

} // namespace guidance