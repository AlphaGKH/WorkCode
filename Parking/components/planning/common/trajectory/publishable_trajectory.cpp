#include "planning/common/trajectory/publishable_trajectory.h"

namespace planning {

PublishableTrajectory::PublishableTrajectory(const double time_stamp,
                                             const DiscretizedTrajectory& discretized_trajectory)
    : DiscretizedTrajectory (discretized_trajectory),
      time_stamp_(time_stamp){}

double PublishableTrajectory::time_stamp() const{
    return time_stamp_;
}

}
