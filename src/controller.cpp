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
    const double psi = eta(5);
    double psi_e = reference.psi_d - psi;

    switch (reference.guidance_mode) {

    case common::GuidanceMode::HeadingHold:
        thrust_reference_ = {0, 0, 0}; // LOL
        break;

    default:
        break;

    }
}



} // namespace controller