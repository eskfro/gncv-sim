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

void Controller::UpdateThrustReference(common::Reference reference, arma::vec6 eta, arma::vec6 nu) {
    const common::ControllerParams p = controller_params_;
    const double psi = eta(5);  // yaw
    const double u = nu(0);     // surge speed 
    const double r = nu(5);     // yaw rate

    switch (reference.guidance_mode) {
        
    case common::GuidanceMode::HeadingHold: {

        // PD heading control
        const double psi_e = common::ssa(reference.psi_d - psi);
        const double N = p.kp_psi * psi_e - p.kd_psi * r;

        // P surge speed control
        const double u_e = reference.u_d - u;
        const double X = p.kp_u * u_e; 

        thrust_reference_ = {X, 0, N};
        break;

    }

    default:
        thrust_reference_ = {0, 0, 0};
        break;

    }
}



} // namespace controller