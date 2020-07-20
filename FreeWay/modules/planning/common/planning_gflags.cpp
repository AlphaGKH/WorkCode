#include "modules/planning/common/planning_gflags.h"

DEFINE_string(planning_config_file,
              "../modules/planning/conf/planning_config.pb.txt",
              "planning config file");

DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");

DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory");

DEFINE_uint64(trajectory_stitching_preserved_length, 20,
              "preserved points number in trajectory stitching");

DEFINE_double(replan_lateral_distance_threshold, 0.5,
              "The lateral distance threshold of replan");
DEFINE_double(replan_longitudinal_distance_threshold, 2.5,
              "The longitudinal distance threshold of replan");

DEFINE_double(longitudinal_acceleration_lower_bound, -4.5,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");

DEFINE_double(speed_lon_decision_horizon, 200.0,
              "Longitudinal horizon for speed decision making (meter)");

DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");

DEFINE_double(planning_upper_speed_limit, 31.3,
              "Maximum speed (m/s) in planning.");

DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(trajectory_space_resolution, 1.0,
              "Trajectory space resolution in planning");

DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in lattice planner.");

DEFINE_double(bound_buffer, 0.1, "buffer to boundary for lateral optimization");
DEFINE_double(nudge_buffer, 0.3, "buffer to nudge for lateral optimization");

DEFINE_bool(lateral_optimization, false,
            "whether using optimization for lateral trajectory generation");
DEFINE_uint64(num_velocity_sample, 6,
              "The number of velocity samples in end condition sampler.");

DEFINE_double(polynomial_minimal_param, 0.01,
              "Minimal time parameter in polynomials.");

DEFINE_double(min_velocity_sample_gap, 1.0,
              "Minimal sampling gap for velocity");

DEFINE_double(time_min_density, 1.0,
              "Minimal time density to search sample points.");

DEFINE_double(default_lon_buffer, 5.0,
              "Default longitudinal buffer to sample path-time points.");

DEFINE_uint64(num_sample_follow_per_timestamp, 3,
              "The number of sample points for each timestamp to follow");

DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 2.0,
              "The upper bound of longitudinal jerk.");

DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");

DEFINE_double(lateral_acceleration_bound, 4.0,
              "Bound of lateral acceleration; symmetric for left and right");

DEFINE_double(lon_collision_buffer, 2.0,
              "The longitudinal buffer to keep distance to other vehicles");
DEFINE_double(lat_collision_buffer, 0.1,
              "The lateral buffer to keep distance to other vehicles");

DEFINE_double(kappa_bound, 0.1979, "The bound for trajectory curvature");

DEFINE_double(lattice_stop_buffer, 0.02,
              "The buffer before the stop s to check trajectories.");

// Lattice Evaluate Parameters
DEFINE_double(weight_lon_objective, 10.0, "Weight of longitudinal travel cost");
DEFINE_double(weight_lon_jerk, 1.0, "Weight of longitudinal jerk cost");
DEFINE_double(weight_lon_collision, 5.0,
              "Weight of longitudinal collision cost");
DEFINE_double(weight_lat_offset, 2.0, "Weight of lateral offset cost");
DEFINE_double(weight_lat_comfort, 10.0, "Weight of lateral comfort cost");
DEFINE_double(weight_centripetal_acceleration, 1.5,
              "Weight of centripetal acceleration");

DEFINE_double(lat_offset_bound, 3.0, "The bound of lateral offset");
DEFINE_double(weight_same_side_offset, 1.0,
              "Weight of same side lateral offset cost");
DEFINE_double(weight_opposite_side_offset, 10.0,
              "Weight of opposite side lateral offset cost");

DEFINE_double(weight_dist_travelled, 10.0, "Weight of travelled distance cost");
DEFINE_double(weight_target_speed, 1.0, "Weight of target speed cost");
DEFINE_double(lon_collision_cost_std, 0.5,
              "The standard deviation of longitudinal collision cost function");
DEFINE_double(lon_collision_yield_buffer, 1.0,
              "Longitudinal collision buffer for yield");
DEFINE_double(lon_collision_overtake_buffer, 5.0,
              "Longitudinal collision buffer for overtake");
DEFINE_double(comfort_acceleration_factor, 0.5,
              "Factor for comfort acceleration.");

DEFINE_double(cost_non_priority_reference_line, 5.0,
              "The cost of planning on non-priority reference line.");

DEFINE_bool(enable_backup_trajectory, true,
            "If generate backup trajectory when planning fail");
DEFINE_double(backup_trajectory_cost, 1000.0,
              "Default cost of backup trajectory");

DEFINE_double(message_latency_threshold, 0.02, "Threshold for message delay");
