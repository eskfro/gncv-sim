#include <armadillo>
#include <cmath>

#include "core/guidance.hpp"
#include "core/common.hpp"

namespace guidance {

// Move this to a config someday
constexpr double T_psi_d = 50;
constexpr double T_u_d = 20;  

void Guidance::Step(double dt) {
    const double t = time_;
    
    switch (guidance_mode_) {

    case common::GuidanceMode::HeadingHold: {

        // Tell which guidance mode to be used
        desired_.guidance_mode = common::GuidanceMode::HeadingHold;

        // Heading reference generation
        const double sine = 60 * std::sin(t * 2 * M_PI / 80);
        const double psi_ref = common::heaviside(t, 90, common::deg2rad(90));

        arma::vec3 x_d = {desired_.psi_d, desired_.r_d, desired_.a_d};
        x_d = common::heading_reference_model(dt, psi_ref, x_d, ref_params_);
        desired_.psi_d = x_d(0);
        desired_.r_d = x_d(1);
        desired_.a_d = x_d(2);

        // Speed reference generation
        const double u_ref = 14 * common::kKnots2Ms;
        desired_.u_d = common::first_order_lowpass(dt, T_u_d, u_ref, desired_.u_d);

        // Position reference (zero in HeadingHold)
        desired_.eta_d.zeros();

        // Integrate time
        time_ += dt;

        break;
    }
    default:
        std::cout << "This should not be printed\n";
    }   
}

} // namespace guidance