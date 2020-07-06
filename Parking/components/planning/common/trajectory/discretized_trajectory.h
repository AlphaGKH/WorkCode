#ifndef PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <vector>

#include "common/math/vec2d.h"
#include "common/proto/pnc_point.pb.h"


namespace planning {

class DiscretizedTrajectory
{
public:
    DiscretizedTrajectory() = default;

    explicit DiscretizedTrajectory(const std::vector<common::TrajectoryPoint>& trajectory_points);

    virtual ~DiscretizedTrajectory() = default;



public:

    virtual uint32_t GetLowerBoundPoint(const double relative_time) const;

    common::TrajectoryPoint StartPoint() const;

    const common::TrajectoryPoint& TrajectoryPointAt(const std::uint32_t index) const;

    virtual uint32_t GetNearestPoint(const common::math::Vec2d& position) const;

    uint32_t NumOfPoints() const {
       return trajectory_points_.size();
   }

public:
    std::vector<common::TrajectoryPoint>& trajectory_points() {
        return trajectory_points_;
    }

    const std::vector<common::TrajectoryPoint>& trajectory_points() const{
      return trajectory_points_;
    }

    virtual void AppendTrajectoryPoint(const common::TrajectoryPoint& trajectory_point);

    template <typename Iter>
    void PrependTrajectoryPoints(Iter begin, Iter end) {
        if (!trajectory_points_.empty() && begin != end) {
            CHECK((end - 1)->relative_time() <
                  trajectory_points_.front().relative_time());
        }
        trajectory_points_.insert(trajectory_points_.begin(), begin, end);
    }

protected:
    std::vector<common::TrajectoryPoint> trajectory_points_;



};

}


#endif
