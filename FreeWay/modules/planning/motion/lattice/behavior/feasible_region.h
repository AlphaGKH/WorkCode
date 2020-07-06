#pragma once

#include <algorithm>

#include "modules/common/proto/pnc_point.pb.h"


namespace dharma {
namespace planning {

class FeasibleRegion {
public:
    explicit FeasibleRegion(const std::array<double, 3>& init_s);

    double SUpper(const double t) const;

    double SLower(const double t) const;

    double VUpper(const double t) const;

    double VLower(const double t) const;

    double TLower(const double s) const;

private:
    std::array<double, 3> init_s_;

    double t_at_zero_speed_;

    double s_at_zero_speed_;
};

}  // namespace planning

}
