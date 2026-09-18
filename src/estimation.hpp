#pragma once

#include <armadillo>

#include "common.hpp"

namespace estimation {

/*
Kalman filter with a constant velocity (CV) update model

In:        2D position measurement
Out:       4D position and velocity estimate
*/
class CvFilter {
public:
    CvFilter() = default;

    void Init(const arma::vec4& x_init, const arma::mat44& P_init);

    void PredictState(double dt);
    void CorrectStateFromMeasurement(const arma::vec2& z);  // z: Gnss measurement

    const arma::vec4& State() const { return x_;}
    const arma::mat44 Cov() const { return P_; }

private:
    arma::vec4 x_{};            // state (x, y, vx, vy)
    arma::mat44 P_{};           // state covariance

    double sigma_a = 0.5;       // process noise stddev
    double sigma_z = 1.0;       // measurement noise stddev

    void GetInitCvState(double dt, arma::vec2& meas0, arma::vec2& meas1);      // init filter from first two measurements

    arma::mat44 F(double dt) const;
    arma::mat44 Q(double dt) const;
    common::mat24 H() const;
    arma::mat22 R() const;

};

/*
Estimate init state from two first measurements on a cv model
*/
void get_init_cv_state(double meas0, double meas1, int ekf_params);

} // namespace estimation