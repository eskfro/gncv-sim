#pragma once

#include <cmath>
#include <armadillo>
#include <cstddef>
#include <string_view>

namespace config {

constexpr double kD2R = M_PI / 180;

struct SimParams {
    double sim_time = 500;
    double dt = 0.025;
};

struct VesselParams {
    // Physical stuff
    double length = 70;
    double breadth = 10;
    double draft = 4.0;                     // gives C_B = nabla / (L*B*T) = 0.65
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
    double gmt = 1.3;                       // transverse metacentric height
    double gml = 100;                       // longitudinal metacentric heights
    double lcf = -5;                        // location of the cf (centre floation) relatice to co

    // Linear damping: [X_u, Y_v, Z_w, K_p, M_q, N_r] (negative -> opposes motion)
    arma::vec6 d_l_coeffs = {-5.0e3, -3.0e5, -2.0e6, -5.0e6, -7.0e7, -4.0e7};
    
    // Quadratic damping: [X_u|u|, Y_v|v|, Z_w|w|, K_p|p|, M_q|q|, N_r|r|]
    arma::vec6 d_n_coeffs = {-5.0e3, -1.4e5, -5.0e5, -1.0e7, -1.0e8, -3.0e8};

    // Diagonal in M_a: [X_u_dot, Y_v_dot, Z_w_dot, K_p_dot, M_q_dot, N_r_dot]
    arma::vec6 added_mass_ratios = {0.08, 1.0, 1.0, 0.2, 1.0, 1.0};

    // Initial conditions
    arma::vec6 eta_0{0, 0, 0, 10 * kD2R, 5 * kD2R, 0};
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

struct MainPropulsorParams {
    std::size_t id = 1;
    std::string_view name = "MainPropulsor";
    arma::vec3 r = {-40, 0, 5};
    arma::vec3 f_dir = {1, 0, 0};
    double time_constant = 2.0;
    double n_max = 220;
    double n_min = -220;
    double k_mp = 16;
};

struct RudderParams {
    std::size_t id = 1;
    std::string_view name = "Rudder";
    arma::vec3 r = {-33, 0, 5};
    arma::vec3 f_dir = {0, 1, 0};
    double time_constant = 2.0;
    double delta_max = 35 * kD2R;
    double delta_min = -35 * kD2R;
    double k_drag = 2e3;
    double k_lift = 7.7e3;
};

struct TunnelThrusterParams {
    std::size_t id = 1;
    std::string_view name = "TunnelThruster";
    arma::vec3 r = {30, 0, 6};
    arma::vec3 f_dir = {0, 1, 0};    
    double time_constant = 2.0;
    double n_max = 500;
    double n_min = -500;
    double k_n = 0.1;     // Force coefficient
    double k_ur = 1;     // Force reduction coefficient for u_r
};



} // namespace config