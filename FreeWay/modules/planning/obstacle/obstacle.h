#pragma once

#include <string>

#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/common/indexed_list.h"


namespace dharma {

namespace planning {

class Obstacle
{
public:
    Obstacle() = default;

public:
    const std::string& Id() const { return id_; }

    const prediction::Trajectory& Trajectory() const { return trajectory_; }

    bool HasTrajectory() const {
        return !(trajectory_.trajectory_point().empty());
    }

    const common::math::Polygon2d& PerceptionPolygon() const {
        return perception_polygon_;
    }

    common::TrajectoryPoint GetPointAtTime(const double relative_time) const;

    common::math::Box2d GetBoundingBox(const common::TrajectoryPoint& point) const;

private:
    std::string id_;

    prediction::Trajectory trajectory_;
    common::math::Polygon2d perception_polygon_;

    perception::PerceptionObstacle perception_obstacle_;
};

typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;


}

}
