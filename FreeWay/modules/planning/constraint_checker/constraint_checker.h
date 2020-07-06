#pragma once

#include "modules/planning/trajectory/discretized_trajectory.h"

namespace dharma {
namespace planning {

class ConstraintChecker {
public:
    enum class Result {
        VALID,
        LON_VELOCITY_OUT_OF_BOUND,
        LON_ACCELERATION_OUT_OF_BOUND,
        LON_JERK_OUT_OF_BOUND,
        LAT_VELOCITY_OUT_OF_BOUND,
        LAT_ACCELERATION_OUT_OF_BOUND,
        LAT_JERK_OUT_OF_BOUND,
        CURVATURE_OUT_OF_BOUND,
    };
    ConstraintChecker() = delete;
    static Result ValidTrajectory(const DiscretizedTrajectory& trajectory);
};

}  // namespace planning
}  // namespace dharma
