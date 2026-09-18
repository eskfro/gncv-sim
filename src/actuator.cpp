#include "actuator.hpp"
#include "common.hpp"
#include <algorithm>
#include <armadillo>

namespace actuators {

// === MAIN PROPULSION ===

void SimulatedMainPropulsion::Step(double dt) {
    n_ += common::first_order_lowpass(dt, time_constant_, n_c_, n_);
}

const arma::vec6& SimulatedMainPropulsion::Tau() {
    const double T = k_mp_ * n_ * std::abs(n_);
    tau_(0) = T;                
    return tau_;  
}

void SimulatedMainPropulsion::SetRpmCommand(double n_c) {
    n_c_ = std::clamp(n_c, n_min_, n_max_);
}

// === RUDDER ===

void SimulatedRudder::Step(double dt) {
    delta_r_ += common::first_order_lowpass(dt, time_constant_, delta_c_, delta_c_);
}

// Simple model for the rudder force
const arma::vec6& SimulatedRudder::Tau(double u) {
    const double F = k_r_ * delta_r_ * u * u;
    tau_(1) = F;                
    tau_(5) = F * l_x_;          
    return tau_;
}

void SimulatedRudder::SetAngleCommand(double delta_c) {
    delta_c_ = std::clamp(delta_c, delta_min, delta_max);
}

// === TUNNEL THRUSTER ===

void SimulatedTunnelThruster::Step(double dt) {
    n_ += common::first_order_lowpass(dt, time_constant_, n_c_, n_);
}

const arma::vec6& SimulatedTunnelThruster::Tau() {
    const double T = k_tt_ * n_ * std::abs(n_);
    tau_(1) = T;
    tau_(5) = T * l_x_;
    return tau_;
}

void SimulatedTunnelThruster::SetRpmCommand(double n_c) {
    n_c_ = std::clamp(n_c, n_min_, n_max_);
}

} // namespace actuators