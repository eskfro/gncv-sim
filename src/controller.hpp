#pragma once

#include "common.hpp"

namespace controller {

class Controller {
public:
    void UpdateThrustReference(double dt, common::Reference reference, arma::vec6 eta, arma::vec6 nu);
    void SetAntiWindupFlags(bool anti_windup_u);

    const arma::vec3& ThrustReference() const { return thrust_reference_; }


private:
    // Anti-windup flags
    bool anti_windup_u_{};

    // States
    double psi_e_{};
    double u_e_int_{};
    double u_e_{};

    // Output
    arma::vec3 thrust_reference_{};

    // Params
    common::ControllerParams controller_params_{};
};

} // namespace controller