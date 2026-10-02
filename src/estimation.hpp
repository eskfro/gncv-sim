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
Error state kalman filter

x = (p, v, q, a_b, w_b) : (position, velocity, orientation, acc bias, gyro bias)
nominal state           : best running guess (dim = 16)
error state             : how strong the nominal state is, this what it estimates (dim = 15)

Why ESKF
Need extra state since covariance matrix dimension is different
because of how attitude is represented as four numbers in a quaternion.
So the error state is one less dimension since we assume
dq = (1, dTheta / 2), which makes P well behaved :)
Also since the error state is often small the linearization
is an accurate representation.

*/
class Eskf15 {
public:
    Eskf15() = default;

    void Init();
    
    // This functions runs everytime a new IMU measurement arrives
    void PredictStateFromImu(double dt, const arma::vec3& acc_meas, const arma::vec3& gyro_meas, const common::ImuParams& p);
    
    // This function runs everytime a position fix arrives, for example Gnss measurement
    void CorrectStateFromGnss(double dt, const arma::vec3& pos_meas, const arma::mat33& R_meas);
    
    
private:
    void InjectErrorState();
    void CovarianceReset();

    // Zero for now ...
    arma::vec3 lever_arm_{};  

    // Nominal state (high rate)
    arma::vec3 p_{};     // IMU position
    arma::vec3 v_{};     // IMU velocity
    common::quat q_{};   // attitude, rotating body to world
    arma::vec3 a_b_{};   // acceleration bias
    arma::vec3 w_b_{};   // gyro bias
    
    
    common::vec15 delta_x_{};
    /*
    0-2     :   delta_p, imu position
    3-5     :   delta_v, imu velocity
    6-8     :   delta_theta, rotation vector
    9-11    :   delta_a_b, acceleration bias
    12-14   :   delta_w_b, gyro bias 
    */

    common::mat1515 P_{};

    // Mounting position correction matrices
    arma::mat33 acc_correction_ = arma::eye(3, 3);
    arma::mat33 gyro_correction_ = arma::eye(3, 3);

};


} // namespace estimation