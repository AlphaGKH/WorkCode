#include "planning/motion/poly_speed_plan/feasible_region.h"

#include <algorithm>

#include "common/mlog/mlog.h"

namespace planning {

FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s,
                               const double& max_acc,
                               const double& max_dec)
    : init_s_(init_s),
      longitudinal_acceleration_upper_bound_(max_acc),
      longitudinal_acceleration_lower_bound_(max_dec) {

    t_at_zero_speed_ = init_s_[1] / (-longitudinal_acceleration_lower_bound_);
    s_at_zero_speed_ = init_s[0] - init_s_[1] * init_s_[1] / (2.0 * longitudinal_acceleration_lower_bound_);
}

double FeasibleRegion::SUpper(const double t) const {
    CHECK(t >= 0.0);
    return init_s_[0] + init_s_[1] * t + 0.5 * longitudinal_acceleration_upper_bound_ * t * t;
}

double FeasibleRegion::SLower(const double t) const {
    if (t < t_at_zero_speed_) {
        return init_s_[0] + init_s_[1] * t +
                0.5 * longitudinal_acceleration_lower_bound_ * t * t;
    }
    return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
    return init_s_[1] + longitudinal_acceleration_upper_bound_ * t;
}

double FeasibleRegion::VLower(const double t) const {
    return t < t_at_zero_speed_ ? init_s_[1] + longitudinal_acceleration_lower_bound_ * t : 0.0;
}

}
