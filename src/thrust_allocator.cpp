#include "thrust_allocator.hpp"
#include "common.hpp"

namespace allocator {

void ThrustAllocator::Step(double dt) {
    /* TODO
    - Low pass filter from reference to command 
    - Function to calculate allocation from force vector
    */
    // Time constants for command-reference dynamics
    double T_r = 0.1;
    double T_tt = 0.1;
    double T_mp = 0.1;

    // commands = H(s) * reference
    const common::ActuatorCommands ref = actuator_references_;
    const common::ActuatorCommands cmd = actuator_commands_;
    actuator_commands_.delta_r += common::first_order_lowpass(dt, T_r, ref.delta_r, cmd.delta_r);
    actuator_commands_.n_tt += common::first_order_lowpass(dt, T_tt, ref.n_tt, cmd.n_tt);
    actuator_commands_.n += common::first_order_lowpass(dt, T_mp, ref.n, cmd.n);


    // Actuator update
    for (auto r : rudders_) {
        r.SetAngleCommand(actuator_commands_.delta_r);
        r.Step(dt);
    }
    for (auto p : main_propulsors_) {
        p.SetRpmCommand(actuator_commands_.n);
        p.Step(dt);
    }
    for (auto t : tunnel_thrusters_) {
        t.SetRpmCommand(actuator_commands_.n_tt);
        t.Step(dt);
    }

}

} // namespace allocator