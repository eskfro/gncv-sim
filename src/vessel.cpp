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
    controller_.UpdateThrustReference(reference_, dynamics_.Eta(), dynamics_.Nu());

    // Actuator allocation
    thrust_allocator_.CalculateActuatorReferences(controller_.ThrustReference(), dynamics_.Nu());
    thrust_allocator_.Step(dt, dynamics_.U());

    // Dynamics update
    tau_ = thrust_allocator_.Tau();
    dynamics_.SetTau(tau_);
    dynamics_.Step(dt);
}

common::VesselSnapshot Vessel::Snapshot() const {
    common::VesselSnapshot s{};
    s.t = dynamics_.Time();
    s.x = dynamics_.State();
    s.tau = dynamics_.Tau();
    return s;
}

} // namespace vessel