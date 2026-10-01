#include "common.hpp"

#include <armadillo>
#include <cmath>

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

    if (!AlmostEqual(common::first_order_lowpass(0.2, 2.0, 10.0, 6.0), 0.4)) return 1;

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
    if (!arma::approx_equal(common::R_quat(q), common::R_zyx(euler), "absdiff", 1e-12)) return 1;

    // Euler round trip
    if (!arma::approx_equal(common::quat_to_euler(q), euler, "absdiff", 1e-12)) return 1;

    // q ⊗ q* = identity
    const common::quat q_qconj = common::quat_mult(q, common::quat_conj(q));
    if (!arma::approx_equal(q_qconj, q_identity, "absdiff", 1e-12)) return 1;

    // Composition matches matrix product: R(q ⊗ p) = R(q)R(p)
    const arma::mat33 R_qp = common::R_quat(common::quat_mult(q, p));
    if (!arma::approx_equal(R_qp, common::R_quat(q) * common::R_quat(p), "absdiff", 1e-12)) return 1;

    // 90 deg about z maps x-axis to y-axis
    const common::quat q_z90 = common::quat_exp(arma::vec3{0.0, 0.0, arma::datum::pi / 2.0});
    const arma::vec3 x_rotated = common::R_quat(q_z90) * arma::vec3{1.0, 0.0, 0.0};
    if (!arma::approx_equal(x_rotated, arma::vec3{0.0, 1.0, 0.0}, "absdiff", 1e-12)) return 1;

    // Small angle branch stays unit length
    const common::quat q_small = common::quat_exp(arma::vec3{1e-10, 0.0, 0.0});
    if (!AlmostEqual(arma::norm(q_small, 2), 1.0)) return 1;

    std::cout << "All tests completed\n";

    return 0;
}
