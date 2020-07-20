#pragma once

#include <string>
#include <list>

#include "modules/common/math/polygon2d.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace dharma {

namespace planning {

class Obstacle {
public:
    Obstacle() = default;

    Obstacle(const std::string &id,
             const perception::PerceptionObstacle &perception_obstacle);

    Obstacle(const std::string &id,
             const perception::PerceptionObstacle &perception_obstacle,
             const prediction::Trajectory &trajectory);

public:
    static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
            const prediction::PredictionObstacles& prediction_obstacles);

    static bool IsValidPerceptionObstacle(const perception::PerceptionObstacle& obstacle);

    static bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

public:
    const std::string &Id() const { return id_; }

    const prediction::Trajectory &Trajectory() const { return trajectory_; }

    bool HasTrajectory() const {
        return !(trajectory_.trajectory_point().empty());
    }

    const common::math::Polygon2d &PerceptionPolygon() const {
        return perception_polygon_;
    }

    common::TrajectoryPoint GetPointAtTime(const double relative_time) const;

    common::math::Box2d GetBoundingBox(const common::TrajectoryPoint &point) const;

private:
    std::string id_; // is not perception_id
    perception::PerceptionObstacle perception_obstacle_;

    common::math::Box2d perception_bounding_box_;
    common::math::Polygon2d perception_polygon_;

    prediction::Trajectory trajectory_;
};

typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

} // namespace planning

} // namespace dharma
