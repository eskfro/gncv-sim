#include "estimation.hpp"
#include <armadillo>
#include <cmath>

namespace estimation {

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