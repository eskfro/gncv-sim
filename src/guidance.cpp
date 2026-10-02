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
    
        reference_.psi_d = common::heaviside(t, 90, common::deg2rad(60));

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