#include "guidance.hpp"
#include "common.hpp"

namespace guidance {

void Guidance::Step(double dt) {
    const int t = time_;
    
    switch (guidance_mode_) {

    case common::GuidanceMode::HeadingHold:

        reference_.guidance_mode = common::GuidanceMode::HeadingHold;
    
        // Heading reference
        if (common::inrange(t, 0, 10)) {
            reference_.psi_d = 10;
        } else if (common::inrange(t, 10, 20)) {
            reference_.psi_d = -20;
        } else {
            reference_.psi_d = 0;
        }

        // Speed reference
        reference_.u_d = 10;
        reference_.eta_d.Reset(); // we dont use this in HeadingHold mode

        // Integrate time
        time_ += dt;

        break;
    
    default:
        std::cout << "This should not be printed\n";
    }   
}

} // namespace guidance