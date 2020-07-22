#pragma once

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/trajectory/discretized_trajectory.h"

namespace dharma {
namespace planning {

class TrajectoryCombiner {
public:
  static DiscretizedTrajectory
  Combine(const std::vector<common::PathPoint> &reference_line,
          const Curve1d &lon_trajectory, const Curve1d &lat_trajectory,
          const double init_relative_time);

private:
  static int index;
};

} // namespace planning
} // namespace dharma
