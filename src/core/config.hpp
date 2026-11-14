#pragma once

#include <cmath>
#include <armadillo>

namespace config {

constexpr double kD2R = M_PI / 180;

struct VesselParams {
    // Physical stuff
    double length = 70;
    double breadth = 10;
    double m = 1.865e6;                     // rho * nabla
    arma::vec3 r_cg = {0, 0, 0};
    arma::vec3 r_cb = {0, 0, -1.3};         // x_cb = x_cg (no trim moment); z_cg - z_cb = GMt
    const double g = 9.81;                  // eple fra tre konstanten
    const double w = m * g;                 // weight
    const double b = w;                     // buoancy

    const double r44 = 3.5;                 // radius of gyration in roll
    const double r55 = 17.5;                // radius of gyration in pitch
    const double r66 = 17.5;                // radius of gyration in yaw
    
    double area_wp = 560;                   // waterplane area (A_wp)
    double nabla = 1820;                    // displacement
    double gmt = 1.3;                        // transverse metacentric height
    double gml = 100;                        // longitudinal metacentric heights
    double lcf = -5;                        // location of the cf (centre floation) relatice to co

    // linear damping: [X_u, Y_v, Z_w, K_p, M_q, N_r] (negative -> opposes motion)
    arma::vec6 d_l_coeffs = {-5.0e3, -3.0e5, -2.0e6, -5.0e6, -7.0e7, -4.0e7};
    
    // quadratic damping: [X_u|u|, Y_v|v|, Z_w|w|, K_p|p|, M_q|q|, N_r|r|]
    arma::vec6 d_n_coeffs = {-5.0e3, -1.4e5, -5.0e5, -1.0e7, -1.0e8, -3.0e8};

    // Initial conditions
    arma::vec6 eta_0{0, 0, 0, 10 * kD2R, 10 * kD2R, 0};
    arma::vec6 nu_0{0, 0, 0, 0, 0, 0};
};

struct ImuParams {
    double sigma_a{};   // acceleration white noise standard deviation
    double sigma_w{};   // gyro white noise standard deviation
    double sigma_aw{};  // acceleration bias driving noise
    double sigma_ww{};  // gyro bias driving noise
    double p_a{};       // inverse time constant acceleration
    double p_w{};       // inverse time constant gyro
    arma::vec3 g = {0, 0, 9.81};    // apple vector
};

struct ControllerParams {
    double kp_psi = 6e6;
    double kd_psi = 6e4;
    double ki_psi = 6e3;
    double kp_u = 6e5;
    double ki_u = 1e4;
};

struct HeadingReferenceModelParams {
    double r_max = 10;
    double a_max = 10;
    double w = 0.03;
    double zeta = 1;
};

struct SurgeSpeedReferenceModelParams {
    double T = 10;
};

// First order. u(s) = h(s) * ref(s) 
struct ActuatorReferenceModelParams {
    double T_r = 0.2;
    double T_tt = 0.2;
    double T_mp = 0.2;
};

} // namespace config