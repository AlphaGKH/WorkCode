#pragma once
#include <list>

#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/obstacle/obstacle.h"
#include "modules/planning/reference_line/reference_line_info.h"

namespace dharma {

namespace planning {

class Frame {
public:
  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state);

  virtual ~Frame() = default;

public:
  bool Init(const std::list<ReferenceLine> &reference_lines);

public:
  std::list<ReferenceLineInfo> *mutable_reference_line_info();

  const std::vector<const Obstacle *> obstacles() const;

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

private:
  bool InitFrameData();

  void AddObstacle(const Obstacle &obstacle);

  const Obstacle *FindCollisionObstacle() const;

  bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines);

private:
  uint32_t sequence_num_ = 0;
  LocalView local_view_;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;

private:
  std::list<ReferenceLineInfo> reference_line_info_;
  ThreadSafeIndexedObstacles obstacles_;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;
};

} // namespace planning
} // namespace dharma
