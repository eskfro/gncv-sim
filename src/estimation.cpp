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


} // namespace estimation