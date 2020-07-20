#pragma once

#include "modules/planning/common/local_view.h"
#include "modules/planning/planner/lattice_planner.h"
#include "modules/planning/trajectory/publishable_trajectory.h"

namespace dharma {

namespace planning {

class Planning {
public:
  Planning() = default;
  ~Planning();

public:
  bool Init();

  bool RunOnce(const LocalView &local_view, std::list<ReferenceLine> ref_lines,
               ADCTrajectory *const adc_trajectory);

private:
  bool InitFrame(const uint32_t sequence_num,
                 const common::TrajectoryPoint &planning_start_point,
                 const common::VehicleState &vehicle_state,
                 std::list<ReferenceLine> ref_lines);

  common::VehicleState AlignTimeStamp(const common::VehicleState &vehicle_state,
                                      const double curr_timestamp) const;

  bool Plan(const double current_time_stamp,
            const std::vector<common::TrajectoryPoint> &stitching_trajectory,
            ADCTrajectory *const trajectory);

private:
  std::unique_ptr<LatticePlanner> planner_;
  size_t seq_num_ = 0;

private:
  LocalView local_view_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
};

} // namespace planning

} // namespace dharma
