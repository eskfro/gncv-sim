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

// The function i implemented in ass. 3 sf from python to c++
// This function is not used and i transferred it to the CvFilter class
void get_init_cv_state(arma::vec2 meas0, arma::vec2 meas1, int ekf_params) {
    double dt;              // TODO: get fro measurement
    arma::vec2 z0 = meas0;  // TODO: make a measurement struct
    arma::vec3 z1 = meas1;
    double sigma_a;         // TODO: ekf_params struct
    double sigma_z;

    arma::mat22 I2 = arma::eye(2, 2);
    arma::mat22 R = std::pow(sigma_z, 2) * I2;

    // Compose process noise matrix
    arma::mat22 Qpp = std::pow(dt, 3) / 3.0 * I2;
    arma::mat22 Qpu = std::pow(dt, 2) / 2.0 * I2;
    arma::mat22 Qup = std::pow(dt, 2) / 2.0 * I2;
    arma::mat22 Quu = dt * I2;
    arma::mat44 Q = arma::join_cols(
        arma::join_rows(Qpp, Qpu),
        arma::join_rows(Qup, Quu)
    ) * std::pow(sigma_a, 2);

    // Compose covariance matrix
    arma::mat22 P11 = R;
    arma::mat22 P12 = (1/dt) * R;
    arma::mat22 P21 = (1/dt) * R;
    arma::mat22 P22 = (2/std::pow(dt, 2))*R + Quu - (2/dt)*Qup + (1/std::pow(dt, 2))*Qpp;
    arma::mat44 P = arma::join_cols(
        arma::join_rows(P11, P12),
        arma::join_rows(P21, P22)
    );

    // Compose mean vector
    arma::vec2 p1_hat = z1;
    arma::vec2 u1_hat = (1/dt) * (z1 - z0);
    arma::vec4 mean;
    mean.subvec(0, 1) = p1_hat;
    mean.subvec(2, 3) = u1_hat;

    // Compose init state
    // init_state = MultiVarGauss(mean, P)
    // return init_state        // TODO: fix return type
}

} // namespace estimation