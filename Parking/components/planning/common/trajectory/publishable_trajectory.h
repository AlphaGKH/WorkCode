#ifndef PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
#define PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_

#include "planning/common/trajectory/discretized_trajectory.h"

namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory
{
public:
    PublishableTrajectory() = default;

    PublishableTrajectory(const double time_stamp,
                          const DiscretizedTrajectory& discretized_trajectory);

    double time_stamp() const;

    void set_time_stamp(const double& time_stamp) {time_stamp_ = time_stamp;}

private:
    double time_stamp_;


};

}


#endif
