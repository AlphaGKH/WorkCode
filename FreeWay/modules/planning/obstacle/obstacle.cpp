#include "modules/planning/obstacle/obstacle.h"

#include "modules/common/math/linear_interpolation.h"

namespace dharma {

namespace planning {

common::TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const{
    const auto& points = trajectory_.trajectory_point();

    if (points.size() < 2) {
        common::TrajectoryPoint point;
        point.mutable_path_point()->set_x(perception_obstacle_.position().x());
        point.mutable_path_point()->set_y(perception_obstacle_.position().y());
        point.mutable_path_point()->set_z(perception_obstacle_.position().z());
        point.mutable_path_point()->set_theta(perception_obstacle_.theta());
        point.mutable_path_point()->set_s(0.0);
        point.mutable_path_point()->set_kappa(0.0);
        point.mutable_path_point()->set_dkappa(0.0);
        point.mutable_path_point()->set_ddkappa(0.0);
        point.set_v(0.0);
        point.set_a(0.0);
        point.set_relative_time(0.0);
        return point;
    }
    else {
        auto comp = [](const common::TrajectoryPoint p, const double time) {
            return p.relative_time() < time;
        };

        auto it_lower =
                std::lower_bound(points.begin(), points.end(), relative_time, comp);

        if (it_lower == points.begin()) {
            return *points.begin();
        } else if (it_lower == points.end()) {
            return *points.rbegin();
        }
        return common::math::InterpolateUsingLinearApproximation(
                    *(it_lower - 1), *it_lower, relative_time);
    }
}

common::math::Box2d Obstacle::GetBoundingBox(const common::TrajectoryPoint& point) const {
    return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                               point.path_point().theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width());
}




}

}
