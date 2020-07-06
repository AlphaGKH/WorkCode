#pragma once

#include <vector>
#include "modules/planning/reference_line/reference_point.h"

namespace dharma {

namespace planning {

class ReferenceLine
{
public:
    ReferenceLine() = default;

public:
    bool GetLaneWidth(const double s, double* const lane_left_width,
                      double* const lane_right_width) const;

    double GetSpeedLimitFromS(const double s) const;

    const std::vector<ReferencePoint>& reference_points() const;

private:
    std::vector<ReferencePoint> reference_points_;
};


}
}
