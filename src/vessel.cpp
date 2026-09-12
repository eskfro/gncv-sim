#include "vessel.hpp"
#include "common.hpp"

namespace vessel {

void Vessel::Step(double dt) {

    for (auto r : rudders_)             r.Step(dt);
    for (auto t : tunnel_thrusters_)    t.Step(dt);
    for (auto p : main_propulsors_)     p.Step(dt);

    guidance_.Step(dt);
    dynamics_.Step(dt);

    reference_ = guidance_.Reference();
    
    




}








} // namespace vessel