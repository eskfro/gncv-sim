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

void Controller::CalculateCommands(common::Reference reference, common::Eta eta, common::Nu nu) {
    const double psi = eta.Psi();
    double psi_e = reference.psi_d - psi;

    switch (reference.guidance_mode) {

    case common::GuidanceMode::HeadingHold:
        commands_.delta = 2 * psi_e;
        break;

    default:
        break;

    }
}



} // namespace controller