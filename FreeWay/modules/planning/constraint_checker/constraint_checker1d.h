#pragma once

#include "modules/planning/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve1d/curve1d.h"

namespace dharma {
namespace planning {

class ConstraintChecker1d {
 public:
  ConstraintChecker1d() = delete;

  static bool IsValidLongitudinalTrajectory(const Curve1d& lon_trajectory);

  static bool IsValidLateralTrajectory(const Curve1d& lat_trajectory,
                                       const Curve1d& lon_trajectory);
};

}  // namespace planning
}  // namespace dharma
