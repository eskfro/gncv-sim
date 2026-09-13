#pragma once

#include "common.hpp"

namespace controller {

class Controller {
public:
    void CalculateActuatorCommands(common::Reference reference, common::Eta eta, common::Nu nu);    

    common::ActuatorCommands ActuatorCommands() { return actuator_commands_; }

private:
    common::ActuatorCommands actuator_commands_{};
};

} // namespace controller