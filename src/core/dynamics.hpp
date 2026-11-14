#pragma once

#include <armadillo>
#include <filesystem>

#include "core/common.hpp"

namespace vessel {

/*
Notes about 6-dof vessel models

- The M matrix can be reduced down to 12 parameters if the vessel is symmetric (port / starboard symmetric) (xz)
- Other symmetries will remove more parameters but are rarely used

Submarine models
- Longitudinal (1, 3, 5) (u_r, w_r, q and theta) 
- Lateral (2, 4, 6) (v_r, p, r, phi, psi)
- We can make a subsystem for longityudinal and lateral equations.
- Then we get M_{long} and M_{lat}

*/

class Dynamics {
public: 
    Dynamics() = default;

    void Step(double dt);
    common::vec12 StateDot(const common::vec12& x, double t) const;

    void UpdateDynamicMatrices();
    void Init(std::filesystem::path vessel_config);     

    // Forces from the vessel fed into dynamics 
    void SetTau(const arma::vec6& tau);

    // Getters
    const arma::vec6& Eta() const { return eta_; }
    const arma::vec6& Nu() const { return nu_; }
    const arma::vec6& Tau() const { return tau_; }
    const common::vec12 State() const { return x_; } 
    double U() const { return common::U(nu_); }
    double Time() const { return t_; };

private:
    double t_{};

    // Vessel params
    config::VesselParams vessel_params_{};

    // State
    common::vec12 x_{};
    arma::vec6 eta_{};
    arma::vec6 nu_{};
    arma::vec6 tau_{};

    // Dynamics
    arma::mat66 D_n_{};     // D_n(v)   : nonlinear damping
    arma::mat66 D_{};       // D(v)     : total damping, D_l + D_n(v)
    arma::mat66 C_rb_{};    // C_RB(v)  : coriolis rigid body
    arma::mat66 C_a_{};     // C_A(v)   : coriolis added mass centripetal
    arma::mat66 C_{};       // C(v)     : total coriolos, C_RB(v) + C_A(v)
    arma::vec6 g_{};        // g(eta)   : restoring forces 
    arma::mat66 J_{};       // J(eta)   : kinematic transform

    arma::mat66 M_rb_{};    // M_RB     : rigid body (mass and intertia tensor)
    arma::mat66 M_a_{};     // M_A      : added mass (hydrodynamics)
    arma::mat66 M_{};       // M        : total mass
    arma::mat66 D_l_{};     // D_l      : linear damping
    arma::mat66 G_{};       // G        : linearized restoring forces
    arma::mat33 I_co_{};    // I_co     : inertia dyadic about r_co = 0
    arma::mat33 I_cg_{};    // I_cg     : inertia dyadic about r_cg

};

} // namespace vessel