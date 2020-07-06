#ifndef PLANNING_MOTION_POLY_SPEED_PLAN_FEASIBLE_REGION_H_
#define PLANNING_MOTION_POLY_SPEED_PLAN_FEASIBLE_REGION_H_

#include <array>

namespace planning {

class FeasibleRegion {
public:
    explicit FeasibleRegion(const std::array<double, 3>& init_s, const double& max_acc, const double& max_dec);

    double SUpper(const double t) const;

    double SLower(const double t) const;

    double VUpper(const double t) const;

    double VLower(const double t) const;

private:
    std::array<double, 3> init_s_;

    double t_at_zero_speed_;

    double s_at_zero_speed_;

private:
    double longitudinal_acceleration_upper_bound_ = 2.0;
    double longitudinal_acceleration_lower_bound_ = -3.0;


};

}


#endif
