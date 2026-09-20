#pragma once

#include "common.hpp"
#include "actuator_io.hpp"

namespace allocator {

/*
In:     Desired force vector from control system
Out:    Actuator commands
*/
class ThrustAllocator {
public:
    ThrustAllocator() = default;

    void Step(double dt, double u);
    void Init();

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
    arma::mat T_alpha_{};       // thrust matrix   :   tau = T(a) * f
    common::ActuatorCommands actuator_references_{};
    common::ActuatorCommands actuator_commands_{};

    // Actuators
    std::vector<actuators_io::Rudder> rudders_{};
    std::vector<actuators_io::MainPropulsion> main_propulsors_{};
    std::vector<actuators_io::TunnelThruster> tunnel_thrusters_{};

};





} // namespace allocator