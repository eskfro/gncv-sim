#include "guidance.hpp"
#include "common.hpp"

namespace guidance {

void Guidance::Step(double dt) {
    const int t = time_;
    
    switch (guidance_mode_) {

    case common::GuidanceMode::HeadingHold:

        reference_.guidance_mode = common::GuidanceMode::HeadingHold;
    
        // Heading reference
        if (common::inrange(t, 0, 120)) {
            reference_.psi_d = common::deg2rad(45);
        } else if (common::inrange(t, 120, 240)) {
            reference_.psi_d = -common::deg2rad(45);
        } else {
            reference_.psi_d = 0;
        }

        // Speed reference
        reference_.u_d = 10;
        reference_.eta_d.zeros(); // we dont use this in HeadingHold mode

        // Integrate time
        time_ += dt;

        break;
    
    default:
        std::cout << "This should not be printed\n";
    }   
}

} // namespace guidance