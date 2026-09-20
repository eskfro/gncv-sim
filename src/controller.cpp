#pragma once
/*
Input:
    Reference from the guidance module

Output: 
    Actuator commands
*/
#include "controller.hpp"
#include "common.hpp"

namespace controller {

void Controller::UpdateThrustReference(double dt, common::Reference reference, arma::vec6 eta, arma::vec6 nu) {
    const common::ControllerParams p = controller_params_;
    const double psi = eta(5);  // yaw
    const double u = nu(0);     // surge speed 
    const double r = nu(5);     // yaw rate

    switch (reference.guidance_mode) {
        
    case common::GuidanceMode::HeadingHold: {

        // PD heading control
        psi_e_ = common::ssa(reference.psi_d - psi);
        const double N = p.kp_psi * psi_e_ - p.kd_psi * r;

        // P surge speed control
        u_e_ = reference.u_d - u;

        if (!anti_windup_u_) {
            u_e_int_ += u_e_ * dt;
        }
        const double X = p.kp_u * u_e_ + p.ki_u * u_e_int_;

        thrust_reference_ = {X, 0, N};
        break;

    }

    default:
        thrust_reference_ = {0, 0, 0};
        break;

    }
}

void Controller::SetAntiWindupFlags(bool anti_windup_u) {
    anti_windup_u_ = anti_windup_u;
}

} // namespace controller