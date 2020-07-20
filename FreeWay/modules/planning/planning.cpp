#include "modules/planning/planning.h"

#include "spider/time/time.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"

#include "modules/map/pnc_map/pnc_map.h"

namespace dharma {

namespace planning {

Planning::~Planning() { EgoInfo::Instance()->Clear(); }

bool Planning::Init() {
  planner_ = std::make_unique<LatticePlanner>();

  if (!planner_) {
    AERROR << "planner is nullptr!";
    return false;
  }
  return true;
}

bool Planning::RunOnce(const LocalView &local_view,
                       std::list<ReferenceLine> ref_lines,
                       ADCTrajectory *const adc_trajectory) {
  const double start_timestamp = spider::Time::Now().ToSecond();
  local_view_ = local_view;

  // 定位信息和底盘信息更新vehicle_state
  bool status = common::VehicleStateProvider::Instance()->Update(
      *local_view_.localization, *local_view_.chassis);

  common::VehicleState vehicle_state =
      common::VehicleStateProvider::Instance()->vehicle_state();

  const double vehicle_state_timestamp = vehicle_state.timestamp();
  DCHECK_GE(start_timestamp, vehicle_state_timestamp);

  if (!status || !util::IsVehicleStateValid(vehicle_state)) {
    AERROR << "Update VehicleStateProvider failed or the vehicle state is out "
              "dated.";
    return false;
  }

  if (start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }

  const double planning_cycle_time = 100.0;

  std::string replan_reason;
  std::vector<common::TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          vehicle_state, start_timestamp, planning_cycle_time,
          FLAGS_trajectory_stitching_preserved_length, true,
          last_publishable_trajectory_.get(), &replan_reason);

  EgoInfo::Instance()->Update(vehicle_state);

  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state,
                     ref_lines);

  if (!status) {
    return false;
  }

  status = Plan(start_timestamp, stitching_trajectory, adc_trajectory);
}

bool Planning::InitFrame(const uint32_t sequence_num,
                         const common::TrajectoryPoint &planning_start_point,
                         const common::VehicleState &vehicle_state,
                         std::list<ReferenceLine> ref_lines) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state));

  if (frame_ == nullptr) {
    AERROR << "Fail to init frame: nullptr.";
    return false;
  }

  auto forward_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

  for (auto &ref_line : ref_lines) {
    if (!ref_line.Segment(
            common::math::Vec2d(vehicle_state.x(), vehicle_state.y()),
            FLAGS_look_backward_distance, forward_limit)) {
      AERROR << "Fail to shrink reference line.";
      return false;
    }
  }

  if (!frame_->Init(ref_lines)) {
    AERROR << "failed to init frame!";
    return false;
  }
  return true;
}

bool Planning::Plan(
    const double current_time_stamp,
    const std::vector<common::TrajectoryPoint> &stitching_trajectory,
    ADCTrajectory *const trajectory) {

  bool status = planner_->Plan(stitching_trajectory.back(), frame_.get());

  const auto *best_ref_info = frame_->FindDriveReferenceLineInfo();

  if (!best_ref_info) {
    AERROR << "planner failed to make a driving plan";
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    return false;
  }

  last_publishable_trajectory_.reset(new PublishableTrajectory(
      current_time_stamp, best_ref_info->trajectory()));

  last_publishable_trajectory_->PrependTrajectoryPoints(
      std::vector<common::TrajectoryPoint>(stitching_trajectory.begin(),
                                           stitching_trajectory.end() - 1));

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory);

  return status;
}

common::VehicleState
Planning::AlignTimeStamp(const common::VehicleState &vehicle_state,
                         const double curr_timestamp) const {
  auto future_xy =
      common::VehicleStateProvider::Instance()->EstimateFuturePosition(
          curr_timestamp - vehicle_state.timestamp());

  common::VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_x(future_xy.x());
  aligned_vehicle_state.set_y(future_xy.y());
  aligned_vehicle_state.set_timestamp(curr_timestamp);

  return aligned_vehicle_state;
}

} // namespace planning

} // namespace dharma
