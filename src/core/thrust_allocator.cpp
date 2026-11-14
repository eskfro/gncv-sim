#include <any>
#include <armadillo>
#include <cstddef>
#include <cstdlib>
#include <vector>

#include "core/thrust_allocator.hpp"
#include "core/actuator_io.hpp"
#include "core/common.hpp"

namespace allocator {

// Fill actuator_io vectors and compose thrust matrix
void ThrustAllocator::Init() {

    // TODO: config
    const auto r1 = actuators_io::Rudder();
    const auto mp1 = actuators_io::MainPropulsion();
    const auto tt1 = actuators_io::TunnelThruster();

    rudders_.push_back(r1);
    main_propulsors_.push_back(mp1);
    tunnel_thrusters_.push_back(tt1);

    num_actuators_ = rudders_.size() + main_propulsors_.size() + tunnel_thrusters_.size();
    actuator_mask_ = arma::vec(num_actuators_, arma::fill::zeros);
    
    // Compose thrust matrix T(a) in 6 DOF.
    // T(a) = [t1, t2, ..., tN] where ti is each actuator.
    int col = 0;
    T_alpha_ = arma::mat(6, num_actuators_, arma::fill::zeros);
    for (const auto& r : rudders_) {
        T_alpha_.col(col) = common::CalculateThrustVector(r.ForceDirection(), r.Position());
        rudder_indices_.push_back(col++);
    }
    for (const auto mp : main_propulsors_) {
        T_alpha_.col(col) = common::CalculateThrustVector(mp.ForceDirection(), mp.Position());
        main_propulsors_indices_.push_back(col++);
    }
    for (const auto tt : tunnel_thrusters_) {
        T_alpha_.col(col) = common::CalculateThrustVector(tt.ForceDirection(), tt.Position());
        tunnel_thruster_indices_.push_back(col++);
    }
}

// Set actuator_references from thrust_vector
void ThrustAllocator::CalculateActuatorReferences(const arma::vec3& thrust_vector, const arma::vec6& nu) {
    // Solve T_alpha * f = thrust_vector
    const arma::uvec dofs = {0, 1, 5};
    
    const double u = common::U(nu);

    // Compose T(a)
    const arma::mat T = T_alpha_.rows(dofs);
    const arma::mat Mask = arma::diagmat(actuator_mask_);

    // Solve
    const arma::vec f = arma::pinv(T * Mask) * thrust_vector;
    
    // Distribute references.
    // u_i = f_i / K_i for each actuator.
    int idx = 0;
    for (auto& r : rudders_)          actuator_references_.delta_r = r.ForceToCommand(f(idx++), u);
    for (auto& mp : main_propulsors_) actuator_references_.n_mp    = mp.ForceToCommand(f(idx++));
    for (auto& tt : tunnel_thrusters_) actuator_references_.n_tt   = tt.ForceToCommand(f(idx++), u);
}

void ThrustAllocator::Step(double dt, double u) {
    // Time constants for firs order command-reference dynamics
    const double T_r = ref_model_params_.T_r;
    const double T_tt = ref_model_params_.T_tt;
    const double T_mp = ref_model_params_.T_mp;

    // commands = H(s) * reference
    const common::ActuatorCommands ref = actuator_references_;
    const common::ActuatorCommands cmd = actuator_commands_;
    actuator_commands_.delta_r = common::first_order_lowpass(dt, T_r, ref.delta_r, cmd.delta_r);
    actuator_commands_.n_tt = common::first_order_lowpass(dt, T_tt, ref.n_tt, cmd.n_tt);
    actuator_commands_.n_mp = common::first_order_lowpass(dt, T_mp, ref.n_mp, cmd.n_mp);

    // Actuator update
    for (auto& r : rudders_) {
        r.SetAngleCommand(actuator_commands_.delta_r);
        r.Step(dt, u);
    }
    for (auto& mp : main_propulsors_) {
        mp.SetRpmCommand(actuator_commands_.n_mp);
        mp.Step(dt, u);
    }
    for (auto& tt : tunnel_thrusters_) {
        tt.SetRpmCommand(actuator_commands_.n_tt);
        tt.Step(dt, u);
    }
}

// tau = sum{i}( [F_i; r x F_i] )
const arma::vec6 ThrustAllocator::Tau() const {
    arma::vec6 tau{};

    // We have tau = Ku. 
    // f = Ku is calculated per actutator in Step().
    for (auto& r : rudders_) { tau += common::CalculateThrustVector(r.Force(), r.Position()); }
    for (auto& mp : main_propulsors_) { tau += common::CalculateThrustVector(mp.Force(), mp.Position()); }
    for (auto& tt : tunnel_thrusters_) { tau += common::CalculateThrustVector(tt.Force(), tt.Position()); }
    return tau;
}

// Ideally maybe calculate max thrust force in -x direction instead ...
bool ThrustAllocator::AntiWindupU() const {
    for (const auto& mp : main_propulsors_) {
        if (common::closeto(mp.Rpm(), mp.MaxRpm())) {
            return true;
        }
    }
    return false;
}

bool ThrustAllocator::RudderSaturated() const {
    for (const auto& r : rudders_) {
        if (common::closeto(std::abs(r.Angle()), r.MaxAngle())) {
            return true;
        }
    }
    return false;
}

void ThrustAllocator::UpdateThrustMode(const common::GuidanceMode& guidance_mode) {
    if (guidance_mode == common::GuidanceMode::HeadingHold) {
        thrust_mode_ = common::ThrustMode::Transit;
        actuator_mask_.zeros();
        for (const size_t& i : rudder_indices_) { actuator_mask_(i) = 1; }
        for (const size_t& i : main_propulsors_indices_) { actuator_mask_(i) = 1; }

    } else if (guidance_mode == common::GuidanceMode::PositionHold) {
        thrust_mode_ = common::ThrustMode::DP;
        actuator_mask_.ones();

    } else {
        common::Panic("Could not update thrust mode.");
    }
}

} // namespace allocator