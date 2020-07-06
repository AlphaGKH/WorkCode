#include "planning/planning.h"

#include <fstream>

#include "common/adapters/adapter_manager.h"
#include "common/util/file.h"

#include "planning/common/gflags_planning.h"
#include "planning/common/trajectory/trajectory_stitcher.h"

#include "planning/LcmMapPathPoint.hpp"
#include "planning/LcmMapPath.hpp"
#include "planning/LcmReferencePoint.hpp"
#include "planning/LcmReferenceLine.hpp"

namespace planning {

Planning::~Planning(){
    if(planning_thread_ptr_ && planning_thread_ptr_->joinable()){
        planning_thread_ptr_->join();
    }
}

bool Planning::Init(){

    road_map_ptr_ = std::make_unique<map::RoadMap>();

    if(!common::util::GetProtoFromFile(FLAGS_planner_conf_file, &planning_config_)){
        AERROR << "Planning can not parse planning config file: " << FLAGS_planner_conf_file;
        return false;
    }

    reference_line_provider_ =
            std::make_unique<ReferenceLineProvider>(road_map_ptr_.get(),
                                                    planning_config_.refline_provider_config());


    if(!common::adapter::AdapterManager::Initialized()){
        if(FLAGS_debug_mode){
            common::adapter::AdapterManager::Init(FLAGS_planning_adapter_conf_debug_file);
        }
        else {
            common::adapter::AdapterManager::Init(FLAGS_planning_adapter_conf_file);
        }
    }

    path_planner_ = std::make_unique<FPPathPlanner>();

    if(!path_planner_->Init(planning_config_.fp_path_plan_config(),
                            planning_config_.ogm_config())){
        AERROR << "path_planner init failed!";
        return false;
    }

    speed_planner_ = std::make_unique<PolySpeedPlanner>();
    if(!speed_planner_->Init(planning_config_.poly_speed_plan_config())){
        AERROR << "speed planner init failed!";
        return false;
    }

    return true;
}

bool Planning::Start(){
    if(!Init()){
        AERROR << "Planning has NOT been initiated.";
        return false;
    }

    if(!reference_line_provider_){
        AERROR << "ReferenceLineProvider is not a valid pointer!";
        return false;
    }

    reference_line_provider_->Start();

    planning_thread_ptr_ = std::make_unique<std::thread>(&Planning::PlanningThread,this);

    return true;

}

void Planning::PlanningThread(){
    const int32_t kPlanningSleepTime = FLAGS_planning_cycle; // ms
    while (!is_stop_) {
        std::this_thread::yield();

        if(last_planning_time_ > kPlanningSleepTime){
            AWARN << "It took Too Long Time for Planning in last cycle!";
        }
        else {
            std::this_thread::sleep_for(
                        std::chrono::duration<double, std::milli>(kPlanningSleepTime - last_planning_time_ * 1000));
        }
        RunOnce();
    }
    return;
}

void Planning::RunOnce(){
    common::adapter::AdapterManager::Observe();

    if(common::adapter::AdapterManager::GetChassisAdapter()->Empty()){
        AWARN << "Chassis not ready";
        return;
    }

    if(common::adapter::AdapterManager::GetGridMapAdapter()->Empty()){
        AWARN << "GridMap not ready";
        return;
    }

    const auto chassis = common::adapter::AdapterManager::GetChassisAdapter()->GetLatestObserved();

    const auto grid_map = common::adapter::AdapterManager::GetGridMapAdapter()->GetLatestObserved();

    common::VehicleState adc_state;
    adc_state.set_x(chassis.x);
    adc_state.set_y(chassis.y);
    adc_state.set_theta(chassis.theta);
    adc_state.set_kappa(chassis.kappa);
    adc_state.set_linear_velocity(chassis.v);
    adc_state.set_linear_acceleration(chassis.a);
    adc_state.set_timestamp(chassis.timestamp);

    ReferenceLine ref_line;
    reference_line_provider_->UpdateVehicleState(adc_state);
    if(!reference_line_provider_->GetReferenceLine(&ref_line)){
        AERROR << "Not get reference_line!";
        return;
    }

    // publish map_path and reference_line for debug
    if(FLAGS_debug_mode)
    {
        LcmMapPath path;
        LcmMapPathPoint lcm_mp;
        path.num_points = ref_line.map_path().num_points();

        for(const auto& mp : ref_line.map_path().points()){
            lcm_mp.x = mp.x();
            lcm_mp.y = mp.y();
            lcm_mp.theta = mp.theta();
            lcm_mp.lane_left_width = mp.lane_left_width();
            lcm_mp.lane_right_width = mp.lane_right_width();

            path.lcm_map_path.emplace_back(lcm_mp);
        }

        LcmReferenceLine line;
        LcmReferencePoint lcm_rp;
        line.num_points = ref_line.reference_points().size();

        for(const auto& rp : ref_line.reference_points()){
            lcm_rp.x = rp.x();
            lcm_rp.y = rp.y();
            lcm_rp.theta = rp.theta();
            lcm_rp.lane_left_width = rp.lane_left_width();
            lcm_rp.lane_right_width = rp.lane_right_width();

            lcm_rp.kappa = rp.kappa();
            lcm_rp.dkappa = rp.dkappa();

            line.lcm_reference_line.emplace_back(lcm_rp);
        }

        common::adapter::AdapterManager::PublishMapPath(path);
        common::adapter::AdapterManager::PublishRefLine(line);
    }

    if(!PlanProcess(adc_state, ref_line, grid_map)){
        return;
    }

    return;
}

bool Planning::PlanProcess(const common::VehicleState &vehicle_state, const ReferenceLine &reference_line,
                           const perception::GridMap& grid_map){

    const double start_timestamp = common::time::Clock::NowInSeconds();

    const double planning_cycle_time = FLAGS_planning_cycle / 1000.0;

    std::vector<common::TrajectoryPoint> stitching_trajectory;

    bool is_replan = false;

    stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
                    vehicle_state, start_timestamp, planning_cycle_time,
                    last_publishable_trajectory_.get(),
                    planning_config_.traj_stitching_config(),
                    &is_replan);


    PathData min_cost_path;
    path_planner_->Process(stitching_trajectory.back(),
                           vehicle_state,
                           reference_line,
                           grid_map,
                           &min_cost_path);
    SpeedData speed_data;
    speed_planner_->Process(stitching_trajectory.back(), min_cost_path, &speed_data);

    DiscretizedTrajectory discretized_trajectory;

    if(!CombinePathAndSpeedProfile(min_cost_path,
                                   speed_data,
                                   stitching_trajectory.back(),
                                   &discretized_trajectory)){
        AERROR << "Combine Path Profile and Speed Profile Failed!";
        return false;
    }

    discretized_trajectory.PrependTrajectoryPoints(stitching_trajectory.begin(),
                                                   stitching_trajectory.end() - 1);

    last_publishable_trajectory_.reset(new PublishableTrajectory(start_timestamp, discretized_trajectory));

    LcmTrajectory lcm_traj;
    LcmTrajectoryPoint point;

    lcm_traj.num_points = discretized_trajectory.trajectory_points().size();
    lcm_traj.timestamp = start_timestamp;
    for(const auto& tp: discretized_trajectory.trajectory_points()){
        point.x = tp.path_point().x();
        point.y = tp.path_point().y();
        point.theta = tp.path_point().theta();
        point.kappa = tp.path_point().kappa();
        point.s = tp.path_point().s();
        point.dkappa = tp.path_point().dkappa();
        point.ddkappa = tp.path_point().ddkappa();
        point.v = tp.v();
        point.a = tp.a();
        point.t = tp.relative_time();
        lcm_traj.lcm_trajectory.push_back(point);
    }


//    std::string fname = "traj.csv";
//    std::fstream openfile(fname, std::ios::ate|std::ios::out);
//    for(const auto& point : discretized_trajectory.trajectory_points()){
//        openfile << point.path_point().x() << " "
//                 << point.path_point().y() << " "
//                 << point.path_point().theta() << " "
//                 << point.path_point().s() << " "
//                 << point.path_point().kappa() << " "
//                 << point.path_point().dkappa() << " "
//                 << point.path_point().ddkappa() << " "
//                 << point.v() << " "
//                 << point.a() << " "
//                 << point.da() << "\n";}

//    openfile.close();


//    common::adapter::AdapterManager::PublishTrajectory(lcm_traj);
    common::adapter::AdapterManager::PublishTrajectory(lcm_traj);

    return true;
}


bool Planning::CombinePathAndSpeedProfile(const PathData &path_data, const SpeedData &speed_data,
                                          const common::TrajectoryPoint &planning_start_point,
                                          DiscretizedTrajectory *discretized_trajectory) {
    const double kDenseTimeResoltuion = 0.02;
    const double kSparseTimeResolution = 0.1;
    const double kDenseTimeSec = 1.0;

    const double relative_time = planning_start_point.relative_time();
    const double start_s = planning_start_point.path_point().s();

    if(path_data.number_points() == 0) {
        return false;
    }

    for(double cur_rel_time = 0.0; cur_rel_time < speed_data.TotalTime(); cur_rel_time +=
        (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion : kSparseTimeResolution)) {

        common::SpeedPoint speed_point;
        if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
            AERROR << "Fail to get speed point with relative time " << cur_rel_time;
            return false;
        }

        if(speed_point.s() > path_data.discretized_path().Length()){
            break;
        }

        common::PathPoint path_point;
        if (!path_data.GetPathPointWithPathS(speed_point.s(), &path_point)) {
            AERROR << "Fail to get path data with s " << speed_point.s()
                   << "path total length " << path_data.discretized_path().Length();
            return false;
        }

        path_point.set_s(path_point.s() + start_s);

        common::TrajectoryPoint trajectory_point;
        trajectory_point.mutable_path_point()->CopyFrom(path_point);
        trajectory_point.set_v(speed_point.v());
        trajectory_point.set_a(speed_point.a());
        trajectory_point.set_relative_time(speed_point.t() + relative_time);

        discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
    }

    return true;
}

void Planning::Stop(){
    AINFO << "Planning Stop is Called!";
    if(reference_line_provider_){
        reference_line_provider_->Stop();
    }

    is_stop_ = true;
    if(planning_thread_ptr_&&planning_thread_ptr_->joinable()){
        planning_thread_ptr_->join();
    }
}


}
