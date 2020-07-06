#pragma once

#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/trajectory/discretized_trajectory.h"

namespace dharma {
namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */
  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const;

  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb) const;

 private:
  double header_time_ = 0.0;
};

}  // namespace planning
}  // namespace dharma
