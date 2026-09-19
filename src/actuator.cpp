#include "actuator.hpp"
#include "common.hpp"
#include <algorithm>
#include <armadillo>
#include <cmath>

namespace actuators {

// === MAIN PROPULSION ===

void SimulatedMainPropulsion::Step(double dt) {
    // Main propulsion dynamics
    n_ += common::first_order_lowpass(dt, time_constant_, n_c_, n_);
    
    // Update tau
    const double T = k_mp_ * n_ * std::abs(n_);
    tau_(0) = T;
}

const arma::vec6& SimulatedMainPropulsion::Tau() const {
    return tau_;  
}

void SimulatedMainPropulsion::SetRpmCommand(double n_c) {
    n_c_ = std::clamp(n_c, n_min_, n_max_);
}

double SimulatedMainPropulsion::ForceToCommand(double f) const {
    return std::copysign(std::sqrt(std::abs(f) / k_mp_), f);
}

// === RUDDER ===

void SimulatedRudder::Step(double dt) {
    delta_r_ += common::first_order_lowpass(dt, time_constant_, delta_c_, delta_r_);
}

void SimulatedRudder::UpdateTau(double u) {
    const double F = k_r_ * delta_r_ * u * u;
    tau_(1) = F;                
    tau_(5) = F * l_x_;   
}

// Simple model for the rudder force
const arma::vec6& SimulatedRudder::Tau() const {
    return tau_;
}

void SimulatedRudder::SetAngleCommand(double delta_c) {
    delta_c_ = std::clamp(delta_c, delta_min_, delta_max_);
}

double SimulatedRudder::ForceToCommand(double f, double u) const {
    // Invert F = k_r * delta * u^2  ->  delta = f / (k_r * u^2)
    constexpr double u_min = 0.1;           // divide by zero guard
    const double u_eff = std::max(std::abs(u), u_min);
    double delta = f / (k_r_ * u_eff * u_eff);

    return std::clamp(delta, -delta_max_, delta_max_);
}

// === TUNNEL THRUSTER ===

void SimulatedTunnelThruster::Step(double dt) {
    // Tunnel thruster dynamics
    n_ += common::first_order_lowpass(dt, time_constant_, n_c_, n_);

    // Update tau
    const double T = k_tt_ * n_ * std::abs(n_);
    tau_(1) = T;
    tau_(5) = T * l_x_;
}

const arma::vec6& SimulatedTunnelThruster::Tau() const {
    return tau_;
}

void SimulatedTunnelThruster::SetRpmCommand(double n_c) {
    n_c_ = std::clamp(n_c, n_min_, n_max_);
}

double SimulatedTunnelThruster::ForceToCommand(double f) const {
    return std::copysign(std::sqrt(std::abs(f) / k_tt_), f);
}

} // namespace actuators