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
    if (common::inrange(0.0, 0.0, 1.0)) return 1;
    if (common::inrange(1.0, 0.0, 1.0)) return 1;

    if (!AlmostEqual(common::ssa(0.0), 0.0)) return 1;
    if (!AlmostEqual(common::ssa(2.0 * arma::datum::pi), 0.0)) return 1;
    if (!AlmostEqual(common::ssa(1.5 * arma::datum::pi), -0.5 * arma::datum::pi)) return 1;

    if (!AlmostEqual(common::first_order_lowpass(0.2, 2.0, 10.0, 6.0), 0.4)) return 1;

    const arma::vec3 a{1.0, 2.0, 3.0};
    const arma::vec3 b{4.0, 5.0, 6.0};
    const arma::vec3 cross_product = arma::cross(a, b);
    const arma::vec3 skew_product = common::S(a) * b;
    if (!arma::approx_equal(cross_product, skew_product, "absdiff", 1e-12)) return 1;

    return 0;
}
