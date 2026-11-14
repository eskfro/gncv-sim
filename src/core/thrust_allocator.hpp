#pragma once

#include "core/common.hpp"
#include "core/actuator_io.hpp"
#include <cstddef>
#include <vector>

namespace allocator {

/*
In:     Desired force vector from control system
Out:    Actuator commands

f: control forces
u: vector of control inputs
K: diagonal force coefficient matrix

(1) f = Ku
(2) tau = Tf = TKu
(3) B = TK
(4) tau = Bu

*/
class ThrustAllocator {
public:
    ThrustAllocator() = default;

    void Step(double dt, double u);
    void Init();

    void UpdateThrustMode(const common::GuidanceMode& guidance_mode);
    void CalculateActuatorReferences(const arma::vec3& thrust_vector, const arma::vec6& nu);

    // Getters
    const arma::vec6 Tau() const;
    bool AntiWindupU() const;
    const common::ActuatorCommands& ActuatorCommands() const { return actuator_commands_; }
    const common::ActuatorCommands& ActuatorReferences() const { return actuator_references_; }
    double DeltaR() const { return rudders_.at(0).Angle(); }
    double RpmMp() const { return main_propulsors_.at(0).Rpm(); }
    double RpmTt() const { return tunnel_thrusters_.at(0).Rpm(); }

private:
    int num_actuators_{};        
    arma::mat T_alpha_{};
    common::ActuatorCommands actuator_references_{};
    common::ActuatorCommands actuator_commands_{};
    common::ThrustMode thrust_mode_{};
    common::ActuatorReferenceModelParams ref_model_params_{};

    // Actuators
    std::vector<actuators_io::Rudder> rudders_{};
    std::vector<actuators_io::MainPropulsion> main_propulsors_{};
    std::vector<actuators_io::TunnelThruster> tunnel_thrusters_{};

    std::vector<size_t> rudder_indices_{};
    std::vector<size_t> main_propulsors_indices_{};
    std::vector<size_t> tunnel_thruster_indices_{};
    arma::vec actuator_mask_{};

};





} // namespace allocator