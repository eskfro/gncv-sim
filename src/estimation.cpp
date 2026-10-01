#include "estimation.hpp"
#include "common.hpp"
#include <armadillo>
#include <cmath>

namespace estimation {

void CvFilter::Init(const arma::vec4& x_init, const arma::mat44& P_init) {
    x_ = x_init;
    P_ = P_init;
}

// The internal prediction model in the filter
void CvFilter::PredictState(double dt) {
    const arma::mat44 F = this->F(dt);      // linear dynamics matrix
    const arma::mat44 Q = this->Q(dt);      // dynamics covariance matrix

    x_ = F * x_;                            // state update
    P_ = F * P_ * F.t() + Q;                // model covariance update
}


// Run filter for a new (x, y) measurement z
void CvFilter::CorrectStateFromMeasurement(const arma::vec2& z) {
    const common::mat24 H = this->H();          // measurement matrix
    const arma::mat22 R = this->R();            // measurement covariance

    const arma::vec2 y = z - H * x_;            // innovation
    const arma::mat22 S = H * P_ * H.t() + R;   // innovation covariance
    const common::mat42 K = P_ * H.t() * S.i(); // kalman gain

    x_ = x_ + K * y;                            // corrected state
    P_ = (arma::eye(4, 4) - K * H) * P_;        // corrected covariance (shrunk) :)

}

// CV model update dynamics 
// x = F * xk + vk
arma::mat44 CvFilter::F(double dt) const {
    return arma::mat44 {
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
}

// Model covariance matrix
// vk ~ N(0, Q)
arma::mat44 CvFilter::Q(double dt) const {
    return std::pow(sigma_a, 2) * arma::mat44 {
        {std::pow(dt,3)/3.0,         0,          std::pow(dt, 2)/2.0,           0},
        {0,         std::pow(dt,3)/3.0,          0,           std::pow(dt, 2)/2.0},
        {std::pow(dt,2)/2.0,         0,          dt,                            0},
        {0,         std::pow(dt,2)/2.0,          0,                            dt}
    };
}

// Measurement matrix
// zk = H * xk + wk
common::mat24 CvFilter::H() const {
    return {
        {1, 0, 0, 0},
        {0, 1, 0, 0}
    };
}

// Measurement covariance matrix
// wk ~ N(0, R)
arma::mat22 CvFilter::R() const {
    return arma::mat22 {
        {std::pow(sigma_z, 2),      0},
        {0,      std::pow(sigma_z, 2)}
    };
}

// Inititate the CV model from the first two measurement
void CvFilter::GetInitCvState(double dt, arma::vec2& pos_meas_1, arma::vec2& pos_meas_2) {
    // Compose init state vector
    arma::vec2 u1_hat = (pos_meas_2 - pos_meas_1) / dt;
    x_.subvec(0, 1) = pos_meas_2;
    x_.subvec(2, 3) = u1_hat;

    // Composes init state covariance matrix
    const arma::mat44 Q = this->Q(dt);
    const arma::mat22 R = this->R();

    const arma::mat22 Q11 = Q.submat(0, 0, 1, 1);
    const arma::mat22 Q21 = Q.submat(2, 0, 3, 1);
    const arma::mat22 Q22 = Q.submat(2, 2, 3, 3);

    const arma::mat22 P11 = R;
    const arma::mat22 P12 = R / dt;
    const arma::mat22 P21 = R / dt;
    const arma::mat22 P22 = (2/std::pow(dt,2))*R + Q11 - (2/dt)*Q21 + (1/std::pow(dt,2))*Q22;

    P_ = common::join22blocks(P11, P12, P21, P22);
}

void Eskf15::PredictState(double dt, const arma::vec3& acc_meas, const arma::vec3& gyro_meas, const common::ImuParams& p) {
    /*
    Uses the latest acceleration and gyro measurements to move the best guess
    of the state forward and works out the new uncertainty.
    
    */
    const arma::mat33 I3 = arma::eye(3, 3);

    // === Nominal state dynamics ===

    // Clean Imu readings and rotate
    arma::vec3 a_body = acc_meas - a_b_;
    arma::vec3 w =  gyro_meas - w_b_;
    arma::mat33 R = common::R_quat(q_);
    arma::vec3 a_world = R * a_body + p.g;

    // Nominal state update
    p_ = p_ + v_ * dt + 0.5 * a_world * std::pow(dt, 2);
    v_ = v_ + a_world * dt;
    q_ = common::quat_mult(q_, common::quat_exp(w * dt));
    q_ = common::quat_normalize(q_);
    a_b_ = std::exp(-p.p_a * dt) * a_b_;
    w_b_ = std::exp(-p.p_w * dt) * w_b_;


    // === Error state dynamics ===

    // delta_x' = A * delta_x + G * n,   n ~ N(0, Q)
    // delta_x = [dp, dv, dtheta, da_b, dw_b]   (15)
    // n       = [a_n, w_n, a_w, w_w]           (12)
    
    common::mat1515 A{}; // Big matrix
    A.submat(0, 3, 2, 5) = I3;
    A.submat(3, 6, 5, 8) = - R * common::S(a_body);
    A.submat(3, 9, 5, 11) = - R;
    A.submat(6, 6, 8, 8) = - common::S(w);
    A.submat(6, 12, 8, 14) = - I3;
    A.submat(9, 9, 11, 11) = - p.p_a * I3;
    A.submat(12, 12, 14, 14) = - p.p_w * I3; 
    
    common::mat1512 G{};
    G.submat(3, 0, 5, 2) = - R;
    G.submat(6, 3, 8, 5) = - I3;
    G.submat(9, 6, 11, 8) = I3;
    G.submat(12, 9, 14, 11) = I3;

    common::mat1212 Q{};
    Q.submat(0, 0, 2, 2) = std::pow(p.sigma_a, 2) * I3;
    Q.submat(3, 3, 5, 5) = std::pow(p.sigma_w, 2) * I3;
    Q.submat(6, 6, 8, 8) = std::pow(p.sigma_aw, 2) * I3;
    Q.submat(9, 9, 11, 11) = std::pow(p.sigma_ww, 2) * I3;

    // === Discretize with Van Loan === 

    constexpr arma::uword n = 15;
    arma::mat V_l(2 * n, 2 * n, arma::fill::zeros);
    V_l.submat(0, 0, n-1, n-1) = -A;
    V_l.submat(0, n, n-1, 2*n-1) = G * Q * G.t();
    V_l.submat(n, n, 2*n-1, 2*n-1) = A.t();

    arma::mat V = arma::expmat(V_l * dt);   // matrix exponential
    
    common::mat1515 Ad = V.submat(n, n, 2*n-1, 2*n-1).t();
    common::mat1515 Qd = Ad * V.submat(0, n, n-1, 2*n-1);

    // === Covariance propagation ===

    P_ = Ad * P_ * Ad.t() + Qd;
    P_ = 0.5 * (P_ + P_.t());   // keep it symmetric
}

void Eskf15::CorrectStateFromMeasurement(double dt, const arma::vec3& pos_meas, const arma::mat33& R_meas) {
    std::cout << "TODO\n";
}

} // namespace estimation