#pragma once

#include "common.hpp"

namespace controller {

class Controller {
public:
    void UpdateThrustReference(common::Reference reference, arma::vec6 eta, arma::vec6 nu);

    const arma::vec3& ThrustReference() const { return thrust_reference_; }

private:
arma::vec3 thrust_reference_{};
common::ControllerParams controller_params_{};
};

} // namespace controller