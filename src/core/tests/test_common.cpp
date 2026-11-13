#include <armadillo>
#include <cmath>

#include "core/common.hpp"

namespace {

bool AlmostEqual(double a, double b, double tol = 1e-12) {
    return std::abs(a - b) <= tol;
}

} // namespace

int main() {
    if (!AlmostEqual(common::deg2rad(180.0), arma::datum::pi)) return 1;
    if (!AlmostEqual(common::deg2rad(-90.0), -arma::datum::pi / 2.0)) return 1;

    if (!common::inrange(0.5, 0.0, 1.0)) return 1;

    if (!AlmostEqual(common::ssa(0.0), 0.0)) return 1;
    if (!AlmostEqual(common::ssa(2.0 * arma::datum::pi), 0.0)) return 1;
    if (!AlmostEqual(common::ssa(1.5 * arma::datum::pi), -0.5 * arma::datum::pi)) return 1;

    // First order lowpass (exact discretization)
    {
        // Single step against closed form: x1 = u + (x0 - u) * exp(-dt/T)
        const double expected = 10.0 + (6.0 - 10.0) * std::exp(-0.2 / 2.0);
        if (!AlmostEqual(common::first_order_lowpass(0.2, 2.0, 10.0, 6.0), expected)) return 1;

        // Steady state: output stays at command
        if (!AlmostEqual(common::first_order_lowpass(0.2, 2.0, 3.0, 3.0), 3.0)) return 1;

        // After one time constant, 1 - exp(-1) (~63.2%) of the step is covered
        if (!AlmostEqual(common::first_order_lowpass(2.0, 2.0, 1.0, 0.0), 1.0 - std::exp(-1.0))) return 1;

        // Many steps match the continuous solution at t = N*dt (exact for constant input)
        const double dt = 0.05, T = 0.7, u = -4.0, x0 = 2.5;
        const int N = 200;
        double x = x0;
        for (int k = 0; k < N; ++k) x = common::first_order_lowpass(dt, T, u, x);
        if (!AlmostEqual(x, u + (x0 - u) * std::exp(-N * dt / T), 1e-9)) return 1;

        // Step size larger than the time constant: no overshoot, ends between curr and command
        const double big = common::first_order_lowpass(10.0, 0.1, 5.0, 0.0);
        if (!(big > 0.0 && big <= 5.0)) return 1;
        if (!AlmostEqual(big, 5.0, 1e-12)) return 1;
    }

    const arma::vec3 a{1.0, 2.0, 3.0};
    const arma::vec3 b{4.0, 5.0, 6.0};
    const arma::vec3 cross_product = arma::cross(a, b);
    const arma::vec3 skew_product = common::S(a) * b;
    if (!arma::approx_equal(cross_product, skew_product, "absdiff", 1e-12)) return 1;

    // Quaternions
    const arma::vec3 euler{0.1, -0.2, 2.5};
    const common::quat q = common::quat_from_euler(euler);
    const common::quat p = common::quat_exp(arma::vec3{0.3, 0.1, -0.4});
    const common::quat q_identity{1.0, 0.0, 0.0, 0.0};

    // Same rotation as the euler angle matrix
    if (!arma::approx_equal(common::quat_to_rotmat(q), common::R_zyx(euler), "absdiff", 1e-12)) return 1;

    // Euler round trip
    if (!arma::approx_equal(common::quat_to_euler(q), euler, "absdiff", 1e-12)) return 1;

    // q ⊗ q* = identity
    const common::quat q_qconj = common::quat_mult(q, common::quat_conj(q));
    if (!arma::approx_equal(q_qconj, q_identity, "absdiff", 1e-12)) return 1;

    // Composition matches matrix product: R(q ⊗ p) = R(q)R(p)
    const arma::mat33 R_qp = common::quat_to_rotmat(common::quat_mult(q, p));
    if (!arma::approx_equal(R_qp, common::quat_to_rotmat(q) * common::quat_to_rotmat(p), "absdiff", 1e-12)) return 1;

    // 90 deg about z maps x-axis to y-axis
    const common::quat q_z90 = common::quat_exp(arma::vec3{0.0, 0.0, arma::datum::pi / 2.0});
    const arma::vec3 x_rotated = common::quat_to_rotmat(q_z90) * arma::vec3{1.0, 0.0, 0.0};
    if (!arma::approx_equal(x_rotated, arma::vec3{0.0, 1.0, 0.0}, "absdiff", 1e-12)) return 1;

    // Small angle branch stays unit length
    const common::quat q_small = common::quat_exp(arma::vec3{1e-10, 0.0, 0.0});
    if (!AlmostEqual(arma::norm(q_small, 2), 1.0)) return 1;

    std::cout << "All tests completed\n";

    return 0;
}
