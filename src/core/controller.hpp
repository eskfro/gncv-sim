#pragma once

#include "core/common.hpp"

namespace controller {

/* Notes from lectures

=== Course controller ===

Beta    : difference between heading angle and U_r vector angle
Beta_c  : crab angle, difference between course and heading angle
X       : course angle, the direction the vessel travels

X = phi + Beta_c

- Might be a good idea to use a kalman filter for estimating the course angle.
- We can use waypoints and a path following algorithm to create course references.
- At stand still we need to use heading control, since vessel have no velocity.
- Gain scheduling, k_p and k_d are functions of U, Prime system 1.
- Use an approximation of the second order Nomoto model.
- We can use the first order Nomoto model to create an autopilot.  

Pivoit point, x_{ng}
- Point on centerline measured from the CG where sway and yaw cancel eachother


=== Heading controller ===

*/
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