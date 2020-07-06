#include "planning/common/trajectory/discretized_trajectory.h"

#include <functional>

#include "common/mlog/mlog.h"


namespace planning {

uint32_t DiscretizedTrajectory::GetLowerBoundPoint(const double relative_time) const{
    CHECK(!trajectory_points_.empty());

    if (relative_time >= trajectory_points_.back().relative_time()) {
        return trajectory_points_.size() - 1;
    }

    auto func = [](const common::TrajectoryPoint& tp, const double relative_time) {
        return tp.relative_time() < relative_time;
    };

    auto iter_lower = std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(), relative_time, func);

    return std::distance(trajectory_points_.begin(), iter_lower);

}

common::TrajectoryPoint DiscretizedTrajectory::StartPoint() const{
    CHECK(!trajectory_points_.empty());
    return trajectory_points_.front();
}

const common::TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(const std::uint32_t index) const{
    CHECK_LT(index, NumOfPoints());
    return trajectory_points_[index];
}

uint32_t DiscretizedTrajectory::GetNearestPoint(const common::math::Vec2d &position) const{
    double dist_sqr_min = std::numeric_limits<double>::max();
    std::uint32_t index_min = 0;
    for (std::uint32_t i = 0; i < trajectory_points_.size(); ++i) {
        const common::math::Vec2d curr_point(
                    trajectory_points_[i].path_point().x(),
                    trajectory_points_[i].path_point().y());

        const double dist_sqr = curr_point.DistanceSquareTo(position);
        if (dist_sqr < dist_sqr_min) {
            dist_sqr_min = dist_sqr;
            index_min = i;
        }
    }
    return index_min;

}

void DiscretizedTrajectory::AppendTrajectoryPoint(const common::TrajectoryPoint &trajectory_point){
    if (!trajectory_points_.empty()) {
        CHECK_GT(trajectory_point.relative_time(),
                 trajectory_points_.back().relative_time());
    }
    trajectory_points_.push_back(trajectory_point);

}

}
