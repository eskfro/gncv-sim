#pragma once

#include "common.hpp"

namespace controller {

class Controller {
public:
    void UpdateThrustReference(common::Reference reference, common::Eta eta, common::Nu nu);    

    const arma::vec3& ThrustVector() const { return thrust_reference_; }

private:
    arma::vec3 thrust_reference_{};
};

} // namespace controller