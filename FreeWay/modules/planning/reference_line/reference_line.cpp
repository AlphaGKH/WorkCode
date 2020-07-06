#include "modules/planning/reference_line/reference_line.h"

#include "modules/planning/common/planning_gflags.h"

namespace dharma {

namespace planning {

bool ReferenceLine::GetLaneWidth(const double s, double *const lane_left_width,
                                 double *const lane_right_width) const{
    return true;
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const{
    double speed_limit = FLAGS_planning_upper_speed_limit;

    return speed_limit;
}

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
    return reference_points_;
}

}

}
