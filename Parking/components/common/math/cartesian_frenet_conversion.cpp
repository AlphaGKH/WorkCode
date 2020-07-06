#include "common/math/cartesian_frenet_conversion.h"

#include <cmath>

#include "common/mlog/mlog.h"
#include "common/math/math_utils.h"

namespace common {
namespace math {

void CartesianFrenetConverter::cartesian_to_frenet(
        const double rs, const double rx, const double ry, const double rtheta,
        const double rkappa, const double rdkappa, const double x, const double y,
        const double v, const double a, const double theta, const double kappa,
        std::array<double, 3>* const ptr_s_condition,
        std::array<double, 3>* const ptr_d_condition) {
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    ptr_d_condition->at(0) =
            std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

    const double delta_theta = theta - rtheta;
    const double tan_delta_theta = std::tan(delta_theta);
    const double cos_delta_theta = std::cos(delta_theta);

    const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
    ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime =
            rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

    ptr_d_condition->at(2) =
            -kappa_r_d_prime * tan_delta_theta +
            one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
            (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition->at(0) = rs;

    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition->at(2) =
            (a * cos_delta_theta -
             ptr_s_condition->at(1) * ptr_s_condition->at(1) *
             (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
            one_minus_kappa_r_d;
}

}  // namespace math
}  // namespace common
