#include "vessel.hpp"
#include "common.hpp"


/*
Architecture

Reference model ---> Heading autopilot ----> Marine craft ---> Gyro, compass -----
                        |                                                         |
                        |                                                         |
                        --------<---------State estimator <-----------<------------
*/

namespace vessel {

void Vessel::Step(double dt) {

    // Guidance update
    guidance_.Step(dt);
    reference_ = guidance_.Reference();
    
    // Dynamics update
    CalculateForces();
    dynamics_.Step(dt);

    // Controller update
    controller_.CalculateActuatorCommands(reference_, dynamics_.Eta(), dynamics_.Nu());
    actuator_commands_ = controller_.ActuatorCommands();

    // Actuator update
    for (auto r : rudders_) {
        r.SetAngleCommand(actuator_commands_.delta);
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

} // namespace vessel