#pragma once

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/trajectory/discretized_trajectory.h"
#include "modules/planning/math/trajectory1d/constant_deceleration_trajectory1d.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/motion/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/math/curve1d/curve1d.h"

namespace dharma {
namespace planning {

class BackupTrajectoryGenerator {
public:
    typedef std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
    Trajectory1dPair;
    typedef std::pair<Trajectory1dPair, double> PairCost;

    BackupTrajectoryGenerator(
            const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
            const double init_relative_time,
            const std::shared_ptr<CollisionChecker>& ptr_collision_checker,
            const Trajectory1dGenerator* trajectory1d_generator);

    DiscretizedTrajectory GenerateTrajectory(
            const std::vector<common::PathPoint>& discretized_ref_points);

private:
    void GenerateTrajectory1dPairs(const std::array<double, 3>& init_s,
                                   const std::array<double, 3>& init_d);

    double init_relative_time_;

    std::shared_ptr<CollisionChecker> ptr_collision_checker_;

    const Trajectory1dGenerator* ptr_trajectory1d_generator_;

    struct CostComparator
            : public std::binary_function<const Trajectory1dPair&,
            const Trajectory1dPair&, bool> {
        bool operator()(const Trajectory1dPair& left,
                        const Trajectory1dPair& right) const {
            auto lon_left = left.first;
            auto lon_right = right.first;
            auto s_dot_left = lon_left->Evaluate(1, FLAGS_trajectory_time_length);
            auto s_dot_right = lon_right->Evaluate(1, FLAGS_trajectory_time_length);
            if (s_dot_left < s_dot_right) {
                return true;
            }
            return false;
        }
    };

    std::priority_queue<Trajectory1dPair, std::vector<Trajectory1dPair>,
    CostComparator>
    trajectory_pair_pqueue_;
};

}  // namespace planning
}  // namespace dharma
