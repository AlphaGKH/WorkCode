#include "modules/planning/frame/frame.h"

#include "modules/common/math/polygon2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/util/util.h"

#include "modules/planning/common/ego_info.h"
#include "modules/planning/obstacle/obstacle.h"

namespace dharma {

namespace planning {

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : sequence_num_(sequence_num), local_view_(local_view),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state) {}

bool Frame::Init(const std::list<ReferenceLine> &reference_lines) {

  if (!InitFrameData()) {
    AERROR << "failed to init frame";
    return false;
  }

  if (!CreateReferenceLineInfo(reference_lines)) {
    AERROR << "Failed to init reference line info.";
    return false;
  }
  return true;
}

bool Frame::InitFrameData() {
  vehicle_state_ = common::VehicleStateProvider::Instance()->vehicle_state();

  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return false;
  }

  for (auto &ptr :
       Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);
  }

  if (planning_start_point_.v() < 1e-3) {
    const auto *collision_obstacle = FindCollisionObstacle();
    if (collision_obstacle != nullptr) {
      std::string err_str =
          "Found vehicle collision with obstacle: " + collision_obstacle->Id();
      AERROR << err_str;
      return false;
    }
  }

  return true;
}

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

const Obstacle *Frame::FindCollisionObstacle() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }

  const auto &adc_polygon =
      common::math::Polygon2d(EgoInfo::Instance()->ego_box());
  for (const auto &obstacle : obstacles_.Items()) {
    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return obstacle;
    }
  }
  return nullptr;
}

bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines) {
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();

  while (ref_line_iter != reference_lines.end()) {
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter);
    ++ref_line_iter;
  }

  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
      AERROR << "Failed to init reference line";
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}

std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

} // namespace planning

} // namespace dharma
