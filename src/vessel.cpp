#include "vessel.hpp"
#include "common.hpp"


/*
Architecture (Fossen) for LOS path following

Reference model ------------> Course autopilot -------------> Marine craft ---------------> Sensors
(GuidanceMode selector)                  |                                                  (Imu, Gnss, Compass)
         |                               |                                                          |
         | (x, y)                        | cog measurements                                         |
         |                               |                                                          |
         |                               |                                                          |   
         --------------------------<---------State estimator <----------------------------------------    
                                            - CV kalman filter                  noisy measuremeants
*/  
namespace vessel {

Vessel::Vessel() {
    dynamics_.Init({});
    thrust_allocator_.Init();
}

void Vessel::Step(double dt) {

    // Guidance update
    guidance_.Step(dt);
    reference_ = guidance_.Reference();
    
    // Controller update
    controller_.SetAntiWindupFlags(thrust_allocator_.AntiWindupU());
    controller_.UpdateThrustReference(dt, reference_, dynamics_.Eta(), dynamics_.Nu());

    // Actuator allocation
    thrust_allocator_.CalculateActuatorReferences(controller_.ThrustReference(), dynamics_.Nu());
    thrust_allocator_.Step(dt, dynamics_.U());

    // Dynamics update
    tau_ = thrust_allocator_.Tau();
    dynamics_.SetTau(tau_);
    dynamics_.Step(dt);
}

common::VesselSnapshot Vessel::Snapshot() const {
    common::ActuatorCommands actuator_states;
    actuator_states.n_mp = thrust_allocator_.RpmMp();
    actuator_states.n_tt = thrust_allocator_.RpmTt();
    actuator_states.delta_r = thrust_allocator_.DeltaR();
    
    common::VesselSnapshot s{};
    s.actuator_references = thrust_allocator_.ActuatorReferences();
    s.actuator_commands = thrust_allocator_.ActuatorCommands();
    s.actuator_states = actuator_states;
    s.reference = guidance_.Reference();
    s.t = dynamics_.Time();
    s.x = dynamics_.State();
    s.tau = dynamics_.Tau();
    return s;
}

} // namespace vessel