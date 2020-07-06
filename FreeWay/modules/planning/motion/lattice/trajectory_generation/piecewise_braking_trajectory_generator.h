#pragma once

#include <memory>

#include "modules/planning/math/trajectory1d/piecewise_acceleration_trajectory1d.h"

namespace dharma {
namespace planning {

class PiecewiseBrakingTrajectoryGenerator {
public:
    PiecewiseBrakingTrajectoryGenerator() = delete;

    static std::shared_ptr<Curve1d> Generate(
            const double s_target, const double s_curr, const double v_target,
            const double v_curr, const double a_comfort, const double d_comfort,
            const double max_time);

    static double ComputeStopDistance(const double v, const double dec);

    static double ComputeStopDeceleration(const double dist, const double v);
};

}  // namespace planning
}  // namespace dharma
