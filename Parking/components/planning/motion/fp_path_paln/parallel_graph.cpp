#include "planning/motion/fp_path_paln/parallel_graph.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <numeric>

#include "common/mlog/mlog.h"
#include "common/util/util.h"
#include "common/util/file.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"
#include "common/visual_handle/visual_handle.h"

#include "planning/common/gflags_planning.h"
#include "perception/common/gflags_perception.h"

namespace planning {

ParallelGraph::ParallelGraph(const FpPathPlanConfig& config, const perception::OgmConfig& ogm_config)
    : fp_config_(config),
      ogm_config_(ogm_config){
    CHECK_GT(fp_config_.sample_config().sample_line_number(), 0);
}


bool ParallelGraph::FindPathTunnel(const common::TrajectoryPoint& init_point,
                                   const common::VehicleState adc_state,
                                   const ReferenceLine& ref_line,
                                   const perception::GridMap& grid_map,
                                   PathData* min_cost_path){
    if(init_point_){
        init_point_->Clear();
    }
    init_point_ = std::make_unique<common::TrajectoryPoint>(init_point);
    ConvertPoint2Another(init_point_->mutable_path_point(), adc_state);

    if(adc_state_){
        adc_state_->Clear();
    }

    adc_state_ = std::make_unique<common::VehicleState>(adc_state);

    // 0. convert reference_line to vehicle coordinate system
    ReferenceLine local_ref_line;
    if(!ConvertReferenceLine2VehicleCoor(ref_line, adc_state, &local_ref_line)){
        return false;
    }

    if(grid_map_){
        grid_map_->array.clear();
    }
    grid_map_ = std::make_unique<perception::GridMap>(grid_map);

    // 1. Sample parallel lines
    std::list<ReferenceLine> parallel_lines;
    if(!SampleParallelLines(local_ref_line, &parallel_lines)){
        return false;
    }

    // 2. kinematic sample path
    std::list<PathData> path_list;
    auto center_first_point = center_local_ref_line_.reference_points().front();

    for(const auto& line : parallel_lines){
        path_list.emplace_back();
        double offset = common::util::DistanceXY(center_first_point, line.reference_points().front());

        if(KinematicSample(line, &path_list.back())){
            if(line.reference_points().front().x() < center_first_point.x()){
                path_list.back().set_offset(-offset);
            }
            else {
                path_list.back().set_offset(offset);
            }
        }
        else {
            path_list.pop_back();
        }
    }

    // 3. extract min_cost_path
    if(!ExtractMinCostPath(path_list, min_cost_path)){
        return false;
    }

    if(FLAGS_enable_visual_debug){
        common::VisualHandle::Init(ogm_config_);
        common::VisualHandle::Visualize(parallel_lines, path_list, grid_map/*, *min_cost_path*/);
    }

    return true;
}

bool ParallelGraph::ConvertReferenceLine2VehicleCoor(const ReferenceLine &raw_ref_line,
                                                     const common::VehicleState &vehicle_pos,
                                                     ReferenceLine* ref_line){
    CHECK_NOTNULL(ref_line);
    if(raw_ref_line.reference_points().empty()){
        return false;
    }

    ref_line->Clear();

    std::vector<ReferencePoint> ref_points;
    ref_points.reserve(raw_ref_line.reference_points().size());

    double vtheta = common::math::NormalizeAngle(vehicle_pos.theta());

    double sin_vtheta = common::math::sin(common::math::Angle16::from_rad(vtheta));
    double cos_vtheta = common::math::cos(common::math::Angle16::from_rad(vtheta));

    double tx = -vehicle_pos.x() * sin_vtheta + vehicle_pos.y() * cos_vtheta;
    double ty = -vehicle_pos.x() * cos_vtheta - vehicle_pos.y() * sin_vtheta;

    std::for_each(raw_ref_line.reference_points().begin(),raw_ref_line.reference_points().end(),
                  [&cos_vtheta, &sin_vtheta, &tx, &ty, &vtheta, &ref_points](const ReferencePoint& rp){
        double new_x = rp.x() * sin_vtheta - rp.y() * cos_vtheta + tx;
        double new_y = rp.x() * cos_vtheta + rp.y() * sin_vtheta + ty;
        double new_theta = common::math::NormalizeAngle(M_PI / 2 + rp.theta() - vtheta);

        ReferencePoint p(rp);
        p.set_x(new_x);
        p.set_y(new_y);
        p.set_theta(new_theta);

        ref_points.push_back(p);
    });

    *ref_line = ReferenceLine(ref_points);

    return true;
}

bool ParallelGraph::SampleParallelLines(const ReferenceLine& local_ref_line, std::list<ReferenceLine>* parallel_lines){
    CHECK_NOTNULL(parallel_lines);
    parallel_lines->clear();

    double adc_left_width = common::VehicleParamHandle::GetParam().left_edge_to_center();
    double adc_right_width = common::VehicleParamHandle::GetParam().right_edge_to_center();

    std::list<std::vector<ReferencePoint>> lateral_points_array;
    for(auto& point : local_ref_line.reference_points()){

        double effictive_left_width = -(point.lane_left_width() - adc_left_width);
        double effictive_right_width = point.lane_right_width() - adc_right_width;

        std::vector<ReferencePoint> lateral_points;
        lateral_points.reserve(fp_config_.sample_config().sample_line_number());

        std::vector<double> sample_l;
        common::util::uniform_slice(effictive_left_width,
                                    effictive_right_width,
                                    static_cast<uint32_t>(fp_config_.sample_config().sample_line_number() - 1),
                                    &sample_l);

        for(uint32_t i = 0; i < fp_config_.sample_config().sample_line_number(); ++i){
            double x = point.x() + sample_l[i] * common::math::sin(common::math::Angle16::from_rad(point.theta()));
            double y = point.y() - sample_l[i] * common::math::cos(common::math::Angle16::from_rad(point.theta()));

            ReferencePoint rp(point);
            rp.set_x(x);
            rp.set_y(y);
            rp.set_lane_left_width(point.lane_left_width() + sample_l[i]);
            rp.set_lane_right_width(point.lane_right_width() - sample_l[i]);
            lateral_points.emplace_back(rp);
        }
        lateral_points_array.emplace_back(lateral_points);
    }

    uint32_t line_size = local_ref_line.reference_points().size();

    for(uint32_t i = 0; i < fp_config_.sample_config().sample_line_number(); ++i){
        std::vector<ReferencePoint> ref_points;
        ref_points.reserve(line_size);

        for(auto iter = lateral_points_array.begin(); iter != lateral_points_array.end(); ++iter){
            ref_points.emplace_back(iter->at(i));
        }

        if(i * 2 == fp_config_.sample_config().sample_line_number() - 1){
            center_local_ref_line_ = ReferenceLine(ref_points);
        }

        parallel_lines->emplace_back(ref_points);
    }

    if(parallel_lines->empty()){
        return false;
    }

    return true;
}

bool ParallelGraph::KinematicSample(const ReferenceLine& ref_line, PathData *smooth_path){
    CHECK_NOTNULL(smooth_path)->Clear();

    uint32_t sample_number =
            static_cast<uint32_t>(fp_config_.sample_config().max_sample_length() / fp_config_.sample_config().sample_interval());

    DiscretizedPath path_points;

    path_points.reserve(sample_number);

    double velocity =
            std::max(FLAGS_top_speed, init_point_->v());

    double kMaxDeltaAngle =
            fp_config_.sample_config().sample_interval() / velocity * common::VehicleParamHandle::GetParam().max_steer_angle_rate();

    common::PathPoint sim_vehicle_point;
    sim_vehicle_point.CopyFrom(init_point_->path_point());
    path_points.emplace_back(sim_vehicle_point);

    double kDistance = std::numeric_limits<double>::infinity();
    int32_t preview_point_index = -1;
    double preview_distance = std::max(2 * adc_state_->linear_velocity(), fp_config_.preview_distance());

    for(const auto& point : ref_line.reference_points()){
        double distance = common::util::DistanceXY(sim_vehicle_point, point);
        if (distance < kDistance)
        {
            kDistance = distance;
            preview_point_index ++;
        }
    }

//    int32_t min_dis_index = preview_point_index;

    if(preview_point_index < 0 || preview_point_index == ref_line.reference_points().size()){
        AERROR << "init_point: " << init_point_->DebugString() << " is too far from reference_line: "
               << ref_line.DebugString();
        return false;
    }

    // stanley sample function

    auto stanly_sample = [this, &ref_line,/* &min_dis_index,*/ &sim_vehicle_point, &velocity]() -> double{
        common::PathPoint front_wheel_point;
        double exp_steer = 0.0;

        front_wheel_point.set_x(sim_vehicle_point.x() +
                                common::VehicleParamHandle::GetParam().wheel_base() * cos(sim_vehicle_point.theta()));
        front_wheel_point.set_y(sim_vehicle_point.y() +
                                common::VehicleParamHandle::GetParam().wheel_base() * sin(sim_vehicle_point.theta()));
        front_wheel_point.set_theta(sim_vehicle_point.theta());

        common::SLPoint sl;
        ref_line.XYToSL({front_wheel_point.x(), front_wheel_point.y()}, &sl);
        auto target_point = ref_line.GetReferencePoint(sl.s());

        double delta_theta = target_point.theta() - sim_vehicle_point.theta();

        delta_theta = common::math::NormalizeAngle(delta_theta);
        double deviation = -sl.l();

        exp_steer = Stanley(delta_theta, deviation, velocity);

        return exp_steer;

    };

    // purepursuit sample function

    auto pure_pursuit_sample = [this, &ref_line, &preview_point_index, &preview_distance,
            &sim_vehicle_point, /*&velocity,*/ &stanly_sample]() -> double {
        double exp_angle = 0;
        for(int32_t i = preview_point_index; i < ref_line.reference_points().size(); ++i){
            const auto& point = ref_line.reference_points().at(i);
            auto temp_d = common::util::DistanceXY(sim_vehicle_point, point);

            if(temp_d >= preview_distance && point.y() > 0){
                preview_point_index = i;
                break;
            }
        }

        auto actual_preview_point =  ref_line.reference_points().at(preview_point_index);

        double actual_preview_distance =
                common::util::DistanceXY(sim_vehicle_point, actual_preview_point);

        if(actual_preview_distance >= fp_config_.preview_distance()){
            ConvertPoint2Another(&actual_preview_point, sim_vehicle_point);

            double x = actual_preview_point.x() * preview_distance/actual_preview_distance;
            double y = actual_preview_point.y() * preview_distance/actual_preview_distance;
            exp_angle = PurePursuit(x, preview_distance);
        }
        else {
            exp_angle = stanly_sample();
        }
    };


    for(uint32_t i = 1; i < sample_number; ++i){
        double last_steer_angle = atan(common::VehicleParamHandle::GetParam().wheel_base() * sim_vehicle_point.kappa());
        double steer_angle = 0;

        if(!fp_config_.only_use_stanley()){
            steer_angle = pure_pursuit_sample();
        }
        else {
            // only use stanley
            steer_angle = stanly_sample();
        }

        // max_steer_angle_rate limit
        if((init_point_->v() > 1e-2) && fp_config_.enable_steer_rate_limit()){
            if(steer_angle > last_steer_angle && steer_angle > last_steer_angle + kMaxDeltaAngle){
                steer_angle = last_steer_angle + kMaxDeltaAngle;
            }

            if(steer_angle < last_steer_angle && steer_angle < last_steer_angle - kMaxDeltaAngle){
                steer_angle = last_steer_angle - kMaxDeltaAngle;
            }
        }

        // max_steer_angle limit
        if (steer_angle > common::VehicleParamHandle::GetParam().max_steer_angle()){
            steer_angle = common::VehicleParamHandle::GetParam().max_steer_angle();
        }
        else if (steer_angle < -common::VehicleParamHandle::GetParam().max_steer_angle()){
            steer_angle = -common::VehicleParamHandle::GetParam().max_steer_angle();
        }

        if(fabs(steer_angle) > 1e-3){
            double radius = common::VehicleParamHandle::GetParam().wheel_base()/tan(steer_angle);
            common::math::Vec2d steering_center;

            steering_center.set_x(sim_vehicle_point.x() - radius * sin(sim_vehicle_point.theta()));
            steering_center.set_y(sim_vehicle_point.y() + radius * cos(sim_vehicle_point.theta()));

            double delta_theta = fp_config_.sample_config().sample_interval() / radius;

            sim_vehicle_point.set_theta(sim_vehicle_point.theta() + delta_theta);

            sim_vehicle_point.set_x(steering_center.x() + radius * sin(sim_vehicle_point.theta()));
            sim_vehicle_point.set_y(steering_center.y() - radius * cos(sim_vehicle_point.theta()));
            sim_vehicle_point.set_kappa((fabs(radius) > 1e-6)? 1/radius:0);
        }
        else {
            common::math::Vec2d p;
            p.set_x(sim_vehicle_point.x() +  fp_config_.sample_config().sample_interval() * cos(sim_vehicle_point.theta()));
            p.set_y(sim_vehicle_point.y() +   fp_config_.sample_config().sample_interval() * sin(sim_vehicle_point.theta()));

            sim_vehicle_point.set_x(p.x());
            sim_vehicle_point.set_y(p.y());
            sim_vehicle_point.set_kappa(0);
        }

        // collision analysis
        if(!CollisionAnalysis(sim_vehicle_point)){ // no collision
            double delta_s = common::util::DistanceXY(path_points.back(), sim_vehicle_point);
            sim_vehicle_point.set_s(path_points.back().s() + delta_s);

            path_points.emplace_back(sim_vehicle_point);
            last_steer_angle = steer_angle;
        }
        else { // has collision
            smooth_path->set_truncated();
            break;
        }

    }

    if(path_points.size() < 2){
        return false;
    }

    smooth_path->SetDiscretizedPath(path_points);

    return true;

}

bool ParallelGraph::ExtractMinCostPath(const std::list<PathData> &path_list,
                                       PathData *min_cost_path){
    CHECK_NOTNULL(min_cost_path);

    if(path_list.empty()){
        return false;
    }

    if(path_list.size() == 1){
        *min_cost_path = path_list.back();
        return true;
    }

    size_t number = path_list.size();

    std::vector<ComparableCost> smooth_costs;
    std::vector<ComparableCost> safety_costs;

    smooth_costs.reserve(number);
    safety_costs.reserve(number);

    CalculateSmoothCost(path_list, &smooth_costs);
    CalculateSafetyCost(path_list, &safety_costs);

    ComparableCost min_cost(true, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    size_t min_index = 0;
    for(size_t i = 0; i < number; ++i){
        ComparableCost total_cost = smooth_costs[i] + safety_costs[i];
        if(total_cost < min_cost){
            min_cost = total_cost;
            min_index = i;
        }
    }

    auto iter = path_list.begin();
    for(size_t j = 0; j < min_index; ++j){
        iter++;
    }

    if(!ConvertPathPoints2WorldCoor(*iter, min_cost_path)){
        return false;
    }

//    *min_cost_path = PathData(*iter);

    return true;
}

void ParallelGraph::CalculateSmoothCost(const std::list<PathData> &path_list,
                                        std::vector<ComparableCost>* smooth_costs){
    CHECK_NOTNULL(smooth_costs);
    common::SLPoint init_sl;
    center_local_ref_line_.XYToSL({init_point_->path_point().x(), init_point_->path_point().y()},
                                  &init_sl);

    std::vector<double> central_costs;
    std::vector<double> lateral_costs;
    double lateral_cost_sum = 0.0;
    double central_cost_sum = 0.0;
    for(auto iter = path_list.begin(); iter != path_list.end(); ++iter){
        double central_cost = iter->Offset() * iter->Offset();
        double lateral_cost = (iter->Offset() + init_sl.l()) * (iter->Offset() + init_sl.l());
        central_cost_sum += central_cost;
        lateral_cost_sum += lateral_cost;
        central_costs.emplace_back(central_cost);
        lateral_costs.emplace_back(lateral_cost);
    }

    size_t number = lateral_costs.size();

    central_cost_sum = std::fabs(central_cost_sum) < 1e-6 ? 1.0 : central_cost_sum;
    lateral_cost_sum = std::fabs(lateral_cost_sum) < 1e-6 ? 1.0 : lateral_cost_sum;

//    if(std::fabs(central_cost_sum) < 1e-6){
//        central_cost_sum = 1.0;
//    }

//    if(std::fabs(lateral_cost_sum) < 1e-6){
//        lateral_cost_sum = 1.0;
//    }

    for(size_t i = 0; i < number; ++i){
        double cost = fp_config_.cost_param().center_cost_weight() * (central_costs[i] / central_cost_sum)
                + fp_config_.cost_param().lateral_cost_weight() * (lateral_costs[i] / lateral_cost_sum);

        smooth_costs->emplace_back(false, cost * fp_config_.cost_param().smooth_cost_weight(), 0.0);
    }
    return;
}

void ParallelGraph::CalculateSafetyCost(const std::list<PathData> &path_list,
                                        std::vector<ComparableCost> *safety_costs){
    CHECK_NOTNULL(safety_costs);
    std::vector<double> lateral_obs_costs;
    std::vector<double> longitudinal_obs_costs;

    bool no_truncated = true;

    double lateral_obs_sum = 0.0;
    double longitudinal_obs_sum = 0.0;

    std::vector<double> offsets;
    std::vector<double> lengths;
    std::vector<bool> truncateds;
    size_t number = path_list.size();
    offsets.reserve(number);
    lengths.reserve(number);
    lateral_obs_costs.reserve(number);
    longitudinal_obs_costs.reserve(number);

    for(auto iter = path_list.begin(); iter != path_list.end(); iter++){
        offsets.emplace_back(iter->Offset());
        lengths.emplace_back(iter->discretized_path().Length());
        truncateds.emplace_back(iter->IsTruncated());
        if(iter->IsTruncated()){
            no_truncated = false;
        }
    }

    if(no_truncated){
        for(size_t m = 0; m < number; ++m){
            safety_costs->emplace_back(false, 0.0, 0.0);
        }
        return;
    }

    for(size_t i = 0; i < number; ++i){
        double lateral_obs_cost = 0;
        double longitudinal_obs_cost = 0;
        double current_offset = offsets[i];
        for(size_t j = 0; j < number; ++j){
            if(truncateds[j]){
                double delta_l = std::fabs(current_offset - offsets[j]);
                if(delta_l <= fp_config_.cost_param().lateral_collision_buffer()){
                    lateral_obs_cost += common::math::Sigmoid(0.5 - delta_l);
                    longitudinal_obs_cost += common::math::Sigmoid(0.5 - lengths[j]);
                }
            }

        }

        lateral_obs_sum += lateral_obs_cost;
        longitudinal_obs_sum += longitudinal_obs_cost;

        lateral_obs_costs.emplace_back(lateral_obs_cost);
        longitudinal_obs_costs.emplace_back(longitudinal_obs_cost);
    }

    if(std::fabs(longitudinal_obs_sum) < 1e-6){
        longitudinal_obs_sum = 1.0;
    }

    if(std::fabs(lateral_obs_sum) < 1e-6){
        lateral_obs_sum = 1.0;
    }


    for(size_t k = 0; k < number; ++k){
        double cost =
                fp_config_.cost_param().lateral_collision_cost_weight() * (lateral_obs_costs[k] / lateral_obs_sum)
                + fp_config_.cost_param().longitudinal_collision_cost_weight()
                * (longitudinal_obs_costs[k] / longitudinal_obs_sum);
        safety_costs->emplace_back(truncateds[k], 0.0, cost * fp_config_.cost_param().safety_cost_weight());
    }

    return;
}

double ParallelGraph::PurePursuit(const double &delta_x, const double &Ld){
    double x = delta_x;
    if(x > Ld){
        x = Ld;
    }
    else if (x < -Ld) {
        x = -Ld;
    }

    return -atan2(2 * common::VehicleParamHandle::GetParam().wheel_base() * x, (Ld * Ld));

}

double ParallelGraph::Stanley(const double& delta_theta, const double& delta_distance, const double& velocity){
    constexpr double K = 0.5;
    return delta_theta + atan(K * delta_distance / velocity);

}

bool ParallelGraph::CollisionAnalysis(const common::PathPoint &point) const{
    int32_t x_cell, y_cell;
    constexpr double resolution = 0.5;

    GetGridMapCoordinate(point.x(), point.y(), &x_cell, &y_cell);

    int32_t last_x_cell = x_cell;
    int32_t last_y_cell = y_cell;

    if(IsOccupied(x_cell, y_cell)){
        return true;                        // collision detected
    }

    double delta_l = resolution;
    while (delta_l <= common::VehicleParamHandle::GetParam().front_edge_to_center()) {
        double x = point.x() + delta_l * cos(point.theta());
        double y = point.y() + delta_l * sin(point.theta());

        GetGridMapCoordinate(x, y, &x_cell, &y_cell);

        if(x_cell != last_x_cell || y_cell != last_y_cell){
            if(IsOccupied(x_cell, y_cell)){
                return true;
            }

            last_x_cell = x_cell;
            last_y_cell = y_cell;
        }

        delta_l += resolution;
    }

    if(std::fabs(delta_l - common::VehicleParamHandle::GetParam().front_edge_to_center()) < 10e-3){
        return false;
    }
    else {
        delta_l = common::VehicleParamHandle::GetParam().front_edge_to_center();
        double x = point.x() + delta_l * cos(point.theta());
        double y = point.y() + delta_l * sin(point.theta());

        GetGridMapCoordinate(x, y, &x_cell, &y_cell);
        if(x_cell != last_x_cell || y_cell != last_y_cell){
            if(IsOccupied(x_cell, y_cell)){
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }
}

void ParallelGraph::GetGridMapCoordinate(const double &x, const double &y, int32_t *x_cell, int32_t *y_cell) const{
    *x_cell = floor((x + ogm_config_.left_width())/ogm_config_.width_resolution());
    *y_cell = floor((ogm_config_.length() - y - ogm_config_.back_lenght())/ogm_config_.length_resolution());

    if(*x_cell < 0){
        *x_cell = 0;
    }

    if(*x_cell + 1 > ogm_config_.width() / ogm_config_.width_resolution()){
        *x_cell = ogm_config_.width() / ogm_config_.width_resolution() - 1;
    }

    if(*y_cell < 0){
        *y_cell = 0;
    }

    if(*y_cell + 1 > ogm_config_.length() / ogm_config_.length_resolution()){
        *y_cell = ogm_config_.length() / ogm_config_.length_resolution() - 1;
    }
}

bool ParallelGraph::IsOccupied(const int32_t &x_cell, const int32_t &y_cell) const{
    // 0: occupied
    if(grid_map_->array[y_cell][x_cell] == 0){
        return true;
    }
    else {
        return false;
    }
}

bool ParallelGraph::ConvertPathPoints2WorldCoor(const PathData& raw_path, PathData* world_path){

    if(raw_path.discretized_path().Length() < 2){
        return false;
    }

    double vx = adc_state_->x();
    double vy = adc_state_->y();
    double vtheta = adc_state_->theta();

    double sin_vtheta = common::math::sin(common::math::Angle16::from_rad(vtheta));
    double cos_vtheta = common::math::cos(common::math::Angle16::from_rad(vtheta));

    DiscretizedPath path;
    path.reserve(raw_path.number_points());

    std::for_each(raw_path.discretized_path().begin(),raw_path.discretized_path().end(),
                  [&cos_vtheta, &sin_vtheta, &vx, &vy, &vtheta, &path](const common::PathPoint& pp){
        double w_x = vx + pp.x() * sin_vtheta + pp.y() * cos_vtheta;
        double w_y = vy - pp.x() * cos_vtheta + pp.y() * sin_vtheta;
        double w_theta = vtheta + pp.theta() - M_PI/2;

        common::PathPoint np;
        np.CopyFrom(pp);
        np.set_x(w_x);
        np.set_y(w_y);
        np.set_theta(w_theta);

        path.emplace_back(np);
    });

    world_path->SetDiscretizedPath(path);

    return true;
}

}
