#include "modules/planning/motion/lattice/lattice_planner.h"

#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/motion/lattice/behavior/prediction_querier.h"
#include "modules/planning/motion/lattice/behavior/path_time_graph.h"
#include "modules/planning/motion/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/motion/lattice/trajectory_generation/trajectory_evaluator.h"
#include "modules/planning/motion/lattice/trajectory_generation/trajectory_combiner.h"
#include "modules/planning/motion/lattice/trajectory_generation/backup_trajectory_generator.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"

namespace dharma {

namespace planning {

namespace {

std::vector<common::PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint>& ref_points) {
    double s = 0.0;
    std::vector<common::PathPoint> path_points;
    for (const auto& ref_point : ref_points) {
        common::PathPoint path_point;
        path_point.set_x(ref_point.x());
        path_point.set_y(ref_point.y());
        path_point.set_theta(ref_point.heading());
        path_point.set_kappa(ref_point.kappa());
        path_point.set_dkappa(ref_point.dkappa());

        if (!path_points.empty()) {
            double dx = path_point.x() - path_points.back().x();
            double dy = path_point.y() - path_points.back().y();
            s += std::sqrt(dx * dx + dy * dy);
        }
        path_point.set_s(s);
        path_points.push_back(std::move(path_point));
    }
    return path_points;
}

void ComputeInitFrenetState(const common::PathPoint& matched_point,
                            const common::TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s, std::array<double, 3>* ptr_d) {
    common::math::CartesianFrenetConverter::cartesian_to_frenet(
                matched_point.s(), matched_point.x(), matched_point.y(),
                matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
                cartesian_state.path_point().x(), cartesian_state.path_point().y(),
                cartesian_state.v(), cartesian_state.a(),
                cartesian_state.path_point().theta(),
                cartesian_state.path_point().kappa(), ptr_s, ptr_d);
}

}  // namespace

bool LatticePlanner::Plan(const common::TrajectoryPoint &planning_init_point, Frame *frame,
                           ADCTrajectory* ptr_computed_trajectory){
    size_t success_line_count = 0;
    size_t index = 0;

    for(auto& reference_line_info : *frame->mutable_reference_line_info()){
        if (index != 0) {
            reference_line_info.SetPriorityCost(FLAGS_cost_non_priority_reference_line);
        }
        else {
            reference_line_info.SetPriorityCost(0.0);
        }

        bool status = PlanOnReferenceLine(planning_init_point, frame, &reference_line_info);

        if(status){
            success_line_count += 1;
        }

        ++index;
    }

    if(success_line_count > 0){
        return true;
    }

    return false;
}

bool LatticePlanner::PlanOnReferenceLine(const common::TrajectoryPoint &planning_init_point, Frame *frame,
                                         ReferenceLineInfo *reference_line_info){
    static size_t num_planning_cycles = 0;
    static size_t num_planning_succeeded_cycles = 0;

    ++num_planning_cycles;

    auto ptr_reference_line =
            std::make_shared<std::vector<common::PathPoint> >(
                ToDiscretizedReferenceLine(reference_line_info->reference_line().reference_points()));

    common::PathPoint matched_point = common::math::PathMatcher::MatchToPath(
                *ptr_reference_line, planning_init_point.path_point().x(),
                planning_init_point.path_point().y());

    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

    auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
                frame->obstacles(), ptr_reference_line);

    auto ptr_path_time_graph =
            std::make_shared<PathTimeGraph>(ptr_prediction_querier-> GetObstacles(),
                                            *ptr_reference_line,
                                            reference_line_info,
                                            init_s[0],
            init_s[0] + FLAGS_speed_lon_decision_horizon,
            0.0,
            FLAGS_trajectory_time_length,
            init_d);

    double speed_limit = reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);

    reference_line_info->SetLatticeCruiseSpeed(speed_limit);

    PlanningTarget planning_target = reference_line_info->planning_target();

    Trajectory1dGenerator trajectory1d_generator(
                init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);

    std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
    std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;

    trajectory1d_generator.GenerateTrajectoryBundles(
                planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

    TrajectoryEvaluator trajectory_evaluator(
                init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
                ptr_path_time_graph, ptr_reference_line);

    CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
            *ptr_reference_line, reference_line_info,
            ptr_path_time_graph);

    size_t collision_failure_count = 0;
    size_t combined_constraint_failure_count = 0;

    size_t lon_vel_failure_count = 0;
    size_t lon_acc_failure_count = 0;
    size_t lon_jerk_failure_count = 0;
    size_t curvature_failure_count = 0;
    size_t lat_acc_failure_count = 0;
    size_t lat_jerk_failure_count = 0;

    size_t num_lattice_traj = 0;

    while (trajectory_evaluator.has_more_trajectory_pairs()) {
        double trajectory_pair_cost =
                trajectory_evaluator.top_trajectory_pair_cost();

        auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

        auto combined_trajectory = TrajectoryCombiner::Combine(
                    *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
                    planning_init_point.relative_time());

        auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
        if (result != ConstraintChecker::Result::VALID) {
            ++combined_constraint_failure_count;
            switch (result) {
            case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
                lon_vel_failure_count += 1;
                break;
            case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
                lon_acc_failure_count += 1;
                break;
            case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
                lon_jerk_failure_count += 1;
                break;
            case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
                curvature_failure_count += 1;
                break;
            case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
                lat_acc_failure_count += 1;
                break;
            case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
                lat_jerk_failure_count += 1;
                break;
            case ConstraintChecker::Result::VALID:
            default:
                // Intentional empty
                break;
            }
            continue;
        }

        if (collision_checker.InCollision(combined_trajectory)) {
            ++collision_failure_count;
            continue;
        }

        num_lattice_traj += 1;

        reference_line_info->SetTrajectory(combined_trajectory);
        reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                     trajectory_pair_cost);
        reference_line_info->SetDrivable(true);

        break;
    }

    if (num_lattice_traj > 0) {
        ADEBUG << "Planning succeeded";
        num_planning_succeeded_cycles += 1;
        reference_line_info->SetDrivable(true);
        return true;
    }
    else {
        AERROR << "Planning failed";
        if (FLAGS_enable_backup_trajectory) {
            AERROR << "Use backup trajectory";
            BackupTrajectoryGenerator backup_trajectory_generator(
                        init_s, init_d, planning_init_point.relative_time(),
                        std::make_shared<CollisionChecker>(collision_checker),
                        &trajectory1d_generator);
            DiscretizedTrajectory trajectory =
                    backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

            reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
            reference_line_info->SetTrajectory(trajectory);
            reference_line_info->SetDrivable(true);
            return true;

        }
        else {
            reference_line_info->SetCost(std::numeric_limits<double>::infinity());
        }

        return false;
    }

}

}

}
