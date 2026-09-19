#include "thrust_allocator.hpp"
#include "actuator_io.hpp"
#include "common.hpp"
#include <armadillo>
#include <cstddef>

namespace allocator {

// Fill actuator_io vectors and compose thrust matrix
void ThrustAllocator::Init() {

    // Compose actuators
    const auto r1 = actuators_io::Rudder();
    const auto mp1 = actuators_io::MainPropulsion();
    const auto tt1 = actuators_io::TunnelThruster();
    rudders_.push_back(r1);
    main_propulsors_.push_back(mp1);
    tunnel_thrusters_.push_back(tt1);
    num_actuators_ = rudders_.size() + main_propulsors_.size() + tunnel_thrusters_.size();
    
    // Compose thrust matrix T(a)
    int col = 0;
    T_alpha_ = arma::mat(3, num_actuators_, arma::fill::zeros);
    for (const auto r : rudders_) {
        T_alpha_.col(col++) = r.ThrustConfig();
    }
    for (const auto mp : main_propulsors_) {
        T_alpha_.col(col++) = mp.ThrustConfig();
    }
    for (const auto tt : tunnel_thrusters_) {
        T_alpha_.col(col++) = tt.ThrustConfig();
    }
}

// Set actuator_references from thrust_vector
void ThrustAllocator::CalculateActuatorReferences(const arma::vec3& thrust_vector, const arma::vec6& nu) {
    // Solve T_alpha * f = thrust_vector
    const arma::vec f = arma::pinv(T_alpha_) * thrust_vector;   // pseudo inverse (over actuated)
    const double u = common::U(nu);

    // Distribute references
    int idx = 0;
    for (auto& r : rudders_)          actuator_references_.delta_r = r.ForceToCommand(f(idx++), u);
    for (auto& mp : main_propulsors_) actuator_references_.n_mp    = mp.ForceToCommand(f(idx++));
    for (auto& tt : tunnel_thrusters_) actuator_references_.n_tt   = tt.ForceToCommand(f(idx++));
}

void ThrustAllocator::Step(double dt, double u) {
    // Time constants for command-reference dynamics
    const double T_r = 0.1;
    const double T_tt = 0.1;
    const double T_mp = 0.1;

    // commands = H(s) * reference
    const common::ActuatorCommands ref = actuator_references_;
    const common::ActuatorCommands cmd = actuator_commands_;
    actuator_commands_.delta_r += common::first_order_lowpass(dt, T_r, ref.delta_r, cmd.delta_r);
    actuator_commands_.n_tt += common::first_order_lowpass(dt, T_tt, ref.n_tt, cmd.n_tt);
    actuator_commands_.n_mp += common::first_order_lowpass(dt, T_mp, ref.n_mp, cmd.n_mp);

    // Actuator update
    for (auto& r : rudders_) {
        r.SetAngleCommand(actuator_commands_.delta_r);
        r.Step(dt);
        r.UpdateTau(u);
    }
    for (auto& mp : main_propulsors_) {
        mp.SetRpmCommand(actuator_commands_.n_mp);
        mp.Step(dt);
    }
    for (auto& tt : tunnel_thrusters_) {
        tt.SetRpmCommand(actuator_commands_.n_tt);
        tt.Step(dt);
    }
}

const arma::vec6 ThrustAllocator::Tau() const {
    arma::vec6 tau{};
    // Actuator update
    for (auto& r : rudders_) {
        tau += r.Tau();
    }
    for (auto& mp : main_propulsors_) {
        tau += mp.Tau();
    }
    for (auto& tt : tunnel_thrusters_) {
        tau += tt.Tau();
    }
    return tau;
} 

} // namespace allocator