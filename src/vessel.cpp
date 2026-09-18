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

void Vessel::Step(double dt) {

    // Guidance update
    guidance_.Step(dt);
    reference_ = guidance_.Reference();
    
    // Dynamics update
    tau_ = thrust_allocator_.Tau(dynamics_.U()); // + wind, wave, hydrodynamic, hydrostatic
    dynamics_.SetTau(tau_);
    dynamics_.Step(dt);

    // Controller update
    controller_.UpdateThrustReference(reference_, dynamics_.Eta(), dynamics_.Nu());

    // Actuator allocation
    thrust_allocator_.CalculateActuatorReferences(controller_.ThrustVector());
    thrust_allocator_.Step(dt);
}

} // namespace vessel