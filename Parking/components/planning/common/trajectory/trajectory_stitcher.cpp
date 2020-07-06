#include "planning/common/trajectory/trajectory_stitcher.h"

#include "common/mlog/mlog.h"
#include "common/math/angle.h"
#include "common/math/math_utils.h"

#include "planning/common/gflags_planning.h"

namespace planning {


std::vector<common::TrajectoryPoint> TrajectoryStitcher::ComputeReinitStitchingTrajectory(
        const common::VehicleState& vehicle_state){
    common::TrajectoryPoint init_point;
    init_point.mutable_path_point()->set_s(0.0);
    init_point.mutable_path_point()->set_x(vehicle_state.x());
    init_point.mutable_path_point()->set_y(vehicle_state.y());
    init_point.mutable_path_point()->set_theta(vehicle_state.theta());
    init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
    init_point.set_v(vehicle_state.linear_velocity());
    init_point.set_a(vehicle_state.linear_acceleration());
    init_point.set_relative_time(0.0);

    return std::vector<common::TrajectoryPoint>(1, init_point);
}

void TrajectoryStitcher::TransformLastPublishedTrajectory(const double x_diff,
                                                          const double y_diff,
                                                          const double theta_diff,
                                                          PublishableTrajectory *prev_trajectory){
    if (!prev_trajectory) {
        return;
    }

    // R^-1
    float cos_theta =
            common::math::cos(common::math::Angle16::from_rad(theta_diff));
    float sin_theta =
            -common::math::sin(common::math::Angle16::from_rad(theta_diff));

    // -R^-1 * t
    auto tx = -(sin_theta * x_diff - cos_theta * y_diff);
    auto ty = -(cos_theta * x_diff + sin_theta * y_diff);

    std::for_each(prev_trajectory->trajectory_points().begin(),
                  prev_trajectory->trajectory_points().end(),
                  [&cos_theta, &sin_theta, &tx, &ty,
                  &theta_diff](common::TrajectoryPoint& p) {
        auto x = p.path_point().x();
        auto y = p.path_point().y();
        auto theta = p.path_point().theta();

        auto x_new = sin_theta * x - cos_theta * y + tx;
        auto y_new = cos_theta * x + sin_theta * y + ty;
        auto theta_new = common::math::WrapAngle(M_PI/2 + theta - theta_diff);

        p.mutable_path_point()->set_x(x_new);
        p.mutable_path_point()->set_y(y_new);
        p.mutable_path_point()->set_theta(theta_new);
    });

}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(const double x, const double y,
                                                                        const common::TrajectoryPoint& p) {
    common::math::Vec2d v(x - p.path_point().x(), y - p.path_point().y());
    common::math::Vec2d n(common::math::cos(common::math::Angle16::from_rad(p.path_point().theta())),
                          common::math::sin(common::math::Angle16::from_rad(p.path_point().theta())));

    std::pair<double, double> frenet_sd;
    frenet_sd.first = v.InnerProd(n) + p.path_point().s();// s
    frenet_sd.second = v.CrossProd(n);// l
    return frenet_sd;
}

std::vector<common::TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(const common::VehicleState &vehicle_state,
                                                                                    const double current_timestamp,
                                                                                    const double planning_cycle_time,
                                                                                    const PublishableTrajectory *prev_trajectory,
                                                                                    const TrajectoryStitchingConfig& config,
                                                                                    bool* is_replan){
    // notice: prev_trajectory is in world coordinate
    *is_replan = true;
    if(!config.enable_trajectory_stitcher() || vehicle_state.linear_velocity() < 0.5){
        return ComputeReinitStitchingTrajectory(vehicle_state);
    }

    if (!prev_trajectory) {
        return ComputeReinitStitchingTrajectory(vehicle_state);
    }

    std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

    if (prev_trajectory_size == 0) {
        return ComputeReinitStitchingTrajectory(vehicle_state);
    }

    // time
    const double vehicle_relative_time =
            current_timestamp - prev_trajectory->time_stamp();

    // behind in current_timestamp
    std::size_t time_matched_index =
            prev_trajectory->GetLowerBoundPoint(vehicle_relative_time);

    if (time_matched_index == 0 &&
            vehicle_relative_time < prev_trajectory->StartPoint().relative_time()) {
        AWARN << "current time smaller than the previous trajectory's first time";
        return ComputeReinitStitchingTrajectory(vehicle_state);
    }

    if (time_matched_index + 1 >= prev_trajectory_size) {
        AWARN << "current time beyond the previous trajectory's last time";
        return ComputeReinitStitchingTrajectory(vehicle_state);
    }

    auto time_matched_point =
            prev_trajectory->TrajectoryPointAt(time_matched_index);

    // space
    std::size_t position_matched_index = prev_trajectory->GetNearestPoint({vehicle_state.x(), vehicle_state.y()});

    auto frenet_sd = ComputePositionProjection(vehicle_state.x(), vehicle_state.y(),
                                               prev_trajectory->TrajectoryPointAt(position_matched_index));

    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;

    if (std::fabs(lat_diff) > config.replan_lateral_distance_threshold() ||
            std::fabs(lon_diff) > config.replan_longitudinal_distance_threshold()) {
        AERROR << "the distance between matched point and actual position is too "
                  "large. Replan is triggered. lat_diff = "
               << lat_diff << ", lon_diff = " << lon_diff;
        return ComputeReinitStitchingTrajectory(vehicle_state);
    }

    double forward_relative_time = vehicle_relative_time + planning_cycle_time;

    std::size_t forward_time_index = prev_trajectory->GetLowerBoundPoint(forward_relative_time);

    auto matched_index = std::min(time_matched_index, position_matched_index);

    std::vector<common::TrajectoryPoint> stitching_trajectory(
                prev_trajectory->trajectory_points().begin()
                + std::max(0, static_cast<int>(matched_index - config.trajectory_stitching_preserved_length())),

                prev_trajectory->trajectory_points().begin() + forward_time_index + 1);

    const double zero_s = stitching_trajectory.back().path_point().s();

    for (auto& tp : stitching_trajectory) {
        tp.set_relative_time(tp.relative_time() + prev_trajectory->time_stamp() -
                             current_timestamp);
        tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
    }
    *is_replan = false;
    // notice: stitching_trajectory is in world coordinate
    return stitching_trajectory;
}


}
