#pragma once

#include "gflags/gflags.h"

DECLARE_string(planning_config_file);
DECLARE_double(default_cruise_speed);

// parameters for trajectory stitching and reinit planning starting point.
DECLARE_bool(enable_trajectory_stitcher);
DECLARE_uint64(trajectory_stitching_preserved_length);

DECLARE_double(replan_lateral_distance_threshold);
DECLARE_double(replan_longitudinal_distance_threshold);

// lattice behavior feasible_region
DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);

// speed sl_boundary
DECLARE_double(speed_lon_decision_horizon);

// parameter for reference_line
DECLARE_double(default_reference_line_width);

// parameters for trajectory planning
DECLARE_double(planning_upper_speed_limit);
DECLARE_double(trajectory_time_length);

// Lattice Planner
DECLARE_double(trajectory_time_resolution);
DECLARE_double(trajectory_space_resolution);
DECLARE_double(numerical_epsilon);

DECLARE_double(bound_buffer);
DECLARE_double(nudge_buffer);

DECLARE_bool(lateral_optimization);

DECLARE_uint64(num_velocity_sample);

DECLARE_double(polynomial_minimal_param);

DECLARE_double(min_velocity_sample_gap);

DECLARE_double(time_min_density);

DECLARE_double(default_lon_buffer);

DECLARE_uint64(num_sample_follow_per_timestamp);

// parameters for trajectory sanity check
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);

DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);

DECLARE_double(lateral_jerk_bound);

DECLARE_double(lateral_acceleration_bound);

DECLARE_double(lon_collision_buffer);
DECLARE_double(lat_collision_buffer);

DECLARE_double(kappa_bound);

DECLARE_double(lattice_stop_buffer);

// Lattice Evaluate Parameters
DECLARE_double(weight_lon_objective);
DECLARE_double(weight_lon_jerk);
DECLARE_double(weight_lon_collision);
DECLARE_double(weight_lat_offset);
DECLARE_double(weight_lat_comfort);
DECLARE_double(weight_centripetal_acceleration);
DECLARE_double(lat_offset_bound);
DECLARE_double(weight_same_side_offset);
DECLARE_double(weight_opposite_side_offset);
DECLARE_double(weight_dist_travelled);
DECLARE_double(weight_target_speed);
DECLARE_double(lon_collision_cost_std);
DECLARE_double(lon_collision_yield_buffer);
DECLARE_double(lon_collision_overtake_buffer);
DECLARE_double(comfort_acceleration_factor);

DECLARE_double(cost_non_priority_reference_line);

DECLARE_bool(enable_backup_trajectory);
DECLARE_double(backup_trajectory_cost);

DECLARE_double(message_latency_threshold);
