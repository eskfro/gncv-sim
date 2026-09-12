#pragma once

#include "common.hpp"

namespace controller {

class Controller {
public:
    void CalculateCommands(common::Reference reference, common::Eta eta, common::Nu nu);    

    common::ActuatorCommands Commands() { return commands_; }

private:
    common::ActuatorCommands commands_{};
};

} // namespace controller