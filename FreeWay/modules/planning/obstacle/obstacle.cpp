#include "modules/planning/obstacle/obstacle.h"

#include "absl/strings/str_cat.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/common/configs/config_gflags.h"

namespace dharma {

namespace planning {

Obstacle::Obstacle(const std::string& id,
                   const perception::PerceptionObstacle& perception_obstacle)
    : id_(id),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                               perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width() ) {
    std::vector<common::math::Vec2d> polygon_points;
    if(FLAGS_use_navigation_mode  || perception_obstacle.polygon_point_size() <= 2){
        perception_bounding_box_.GetAllCorners(&polygon_points);
    }
    else {
        CHECK(perception_obstacle.polygon_point_size() > 2)
                << "object " << id << "has less than 3 polygon points";
        for (const auto& point : perception_obstacle.polygon_point()) {
            polygon_points.emplace_back(point.x(), point.y());
        }
    }

    CHECK(common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                     &perception_polygon_))
            << "object[" << id << "] polygon is not a valid convex hull.\n"
            << perception_obstacle.DebugString();
}

Obstacle::Obstacle(const std::string& id,
                   const perception::PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory)
    : Obstacle(id, perception_obstacle){
    trajectory_ = trajectory;
    auto& trajectory_points = *trajectory_.mutable_trajectory_point();
    double cumulative_s = 0.0;
    if (trajectory_points.size() > 0) {
        trajectory_points[0].mutable_path_point()->set_s(0.0);
    }
    for (int i = 1; i < trajectory_points.size(); ++i) {
        const auto& prev = trajectory_points[i - 1];
        const auto& cur = trajectory_points[i];
        if (prev.relative_time() >= cur.relative_time()) {
            AERROR << "prediction time is not increasing."
                   << "current point: " << cur.ShortDebugString()
                   << "previous point: " << prev.ShortDebugString();
        }
        cumulative_s +=
                common::util::DistanceXY(prev.path_point(), cur.path_point());
        trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
    }
}

std::list<std::unique_ptr<Obstacle> > Obstacle::CreateObstacles(
        const prediction::PredictionObstacles &prediction_obstacles){
    std::list<std::unique_ptr<Obstacle>> obstacles;
    for(const auto& prediction_obstacle : prediction_obstacles.prediction_obstacle()) {
        if (!IsValidPerceptionObstacle(prediction_obstacle.perception_obstacle())) {
            AERROR << "Invalid perception obstacle: "
                   << prediction_obstacle.perception_obstacle().DebugString();
            continue;
        }

        const auto perception_id =
                std::to_string(prediction_obstacle.perception_obstacle().id());

        if(prediction_obstacle.trajectory().empty()) {
            obstacles.emplace_back(
                        new Obstacle(perception_id, prediction_obstacle.perception_obstacle()));
            continue;
        }

        int trajectory_index = 0;

        for(const auto& trajectory : prediction_obstacle.trajectory()) {
            bool is_valid_trajectory = true;
            for(const auto& point : trajectory.trajectory_point()){
                if (!IsValidTrajectoryPoint(point)) {
                    AERROR << "obj:" << perception_id
                           << " TrajectoryPoint: " << trajectory.ShortDebugString()
                           << " is NOT valid.";
                    is_valid_trajectory = false;
                    break;
                }
            }

            if(!is_valid_trajectory){
                continue;
            }

            const std::string obstacle_id =
                    absl::StrCat(perception_id, "_", trajectory_index);

            obstacles.emplace_back(
                        new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle(), trajectory));
            ++trajectory_index;
        }
    }
    return obstacles;
}

bool Obstacle::IsValidPerceptionObstacle(const perception::PerceptionObstacle &obstacle) {
    if (obstacle.length() <= 0.0) {
        AERROR << "invalid obstacle length:" << obstacle.length();
        return false;
    }
    if (obstacle.width() <= 0.0) {
        AERROR << "invalid obstacle width:" << obstacle.width();
        return false;
    }
    if (obstacle.height() <= 0.0) {
        AERROR << "invalid obstacle height:" << obstacle.height();
        return false;
    }

    for (auto pt : obstacle.polygon_point()) {
        if (std::isnan(pt.x()) || std::isnan(pt.y())) {
            AERROR << "invalid obstacle polygon point:" << pt.DebugString();
            return false;
        }
    }
    return true;

}

bool Obstacle::IsValidTrajectoryPoint(const common::TrajectoryPoint& point) {
    return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
             std::isnan(point.path_point().y()) ||
             std::isnan(point.path_point().z()) ||
             std::isnan(point.path_point().kappa()) ||
             std::isnan(point.path_point().s()) ||
             std::isnan(point.path_point().dkappa()) ||
             std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
             std::isnan(point.a()) || std::isnan(point.relative_time()));
}

common::TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const {
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
