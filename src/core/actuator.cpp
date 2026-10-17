#include <algorithm>
#include <armadillo>
#include <cmath>
#include <cstdlib>

#include "core/actuator.hpp"
#include "core/common.hpp"

namespace actuators {

// === MAIN PROPULSION ===

void SimulatedMainPropulsion::Step(double dt, double u_r) {
    // Main propulsion dynamics
    n_ += common::first_order_lowpass(dt, time_constant_, n_c_, n_);
    
    // Force
    const double f_x = k_mp_ * n_ * std::abs(n_);
    f_vec_ = {f_x, 0, 0};

}

void SimulatedMainPropulsion::SetRpmCommand(double n_c) {
    n_c_ = std::clamp(n_c, n_min_, n_max_);
}

double SimulatedMainPropulsion::ForceToCommand(double f) const {
    return common::sign(f) * std::sqrt(std::abs(f) / k_mp_);
}

// === RUDDER ===

void SimulatedRudder::Step(double dt, double u_r) {
    // Dynamics
    const double delta_r_prev = delta_r_;
    delta_r_ += common::first_order_lowpass(dt, time_constant_, delta_c_, delta_r_);
    delta_r_rate_ = (delta_r_ - delta_r_prev) / dt;

    const double u2s = u_r * std::abs(u_r);
    const double delta2 = std::pow(delta_r_, 2);
    const double delta = delta_r_;

    // Force vector
    const double f_x =  -k_drag_ * u2s * delta2;
    const double f_y =   k_lift_ * u2s * delta;
    const double f_z =   0;

    f_vec_ = {f_x, f_y, f_z};

}

void SimulatedRudder::SetAngleCommand(double delta_c) {
    delta_c_ = std::clamp(delta_c, delta_min_, delta_max_);
}

double SimulatedRudder::ForceToCommand(double f, double u) const {
    constexpr double u_min = 0.1;           // divide by zero guard
    const double sign = common::sign(u);
    const double u_r = std::max(std::abs(u), u_min);

    double delta = sign * f / (k_lift_ * u_r * u_r);

    return std::clamp(delta, -delta_max_, delta_max_);
}

// === TUNNEL THRUSTER ===

void SimulatedTunnelThruster::Step(double dt, double u_r) {
    const double u = std::abs(u_r);
    const double n_prev = n_;
    // Tunnel thruster dynamics
    n_ += common::first_order_lowpass(dt, time_constant_, n_c_, n_);
    n_rate_ = (n_ - n_prev) / dt;

    // Update force vector
    const double decay = std::exp(-u*k_ur_);
    const double force =  decay * k_tt_ * n_ * std::abs(n_);
    f_vec_ = force * f_dir_;
}

void SimulatedTunnelThruster::SetRpmCommand(double n_c) {
    n_c_ = std::clamp(n_c, n_min_, n_max_);
}

double SimulatedTunnelThruster::ForceToCommand(double f, double u_r) const {
    const double u = std::abs(u_r);
    const double sign = common::sign(f);
    const double decay = std::exp(-u*k_ur_);

    return sign * std::sqrt(std::abs(f) / (k_tt_ * decay)); 
}

} // namespace actuators