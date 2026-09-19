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

    const arma::vec6 Tau() const;

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