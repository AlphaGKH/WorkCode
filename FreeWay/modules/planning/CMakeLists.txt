project(planning LANGUAGES CXX) 

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})

# lattice
set(LATTICE_SRCS
# common
    ../common/proto/pnc_point.pb.cc
    ../common/proto/geometry.pb.cc
    ../common/proto/header.pb.cc
    ../common/proto/vehicle_param.pb.cc

    ../common/math/aabox2d.cpp
    ../common/math/box2d.cpp
    ../common/math/linear_interpolation.cpp
    ../common/math/line_segment2d.cpp
    ../common/math/math_utils.cpp
    ../common/math/path_matcher.cpp
    ../common/math/polygon2d.cpp
    ../common/math/vec2d.cpp
    ../common/math/cartesian_frenet_conversion.cpp

    ../common/util/string_util.cpp

    ../common/vehicle_param_handle/vehicle_param_handle.cpp
# canbus
    ../canbus/proto/chassis.pb.cc
# perception
    ../perception/proto/perception_obstacle.pb.cc
# prediction
    ../prediction/proto/feature.pb.cc
    ../prediction/proto/prediction_obstacle.pb.cc
# planning
    proto/lattice_structure.pb.cc
    proto/planning.pb.cc

    common/planning_gflags.cpp

    obstacle/obstacle.cpp

    reference_line/reference_point.cpp
    reference_line/reference_line.cpp
    reference_line/reference_line_info.cpp

    trajectory/discretized_trajectory.cpp
    trajectory/publishable_trajectory.cpp

    math/curve1d/quartic_polynomial_curve1d.cpp
    math/curve1d/quintic_polynomial_curve1d.cpp
    math/trajectory1d/piecewise_acceleration_trajectory1d.cpp
    math/trajectory1d/constant_deceleration_trajectory1d.cpp

    constraint_checker/constraint_checker1d.cpp
    constraint_checker/collision_checker.cpp
    constraint_checker/constraint_checker.cpp
    frame/frame.cpp

    motion/lattice/behavior/feasible_region.cpp
    motion/lattice/behavior/prediction_querier.cpp
    motion/lattice/behavior/path_time_graph.cpp
    motion/lattice/trajectory_generation/end_condition_sampler.cpp
    motion/lattice/trajectory_generation/lattice_trajectory1d.cpp
    motion/lattice/trajectory_generation/trajectory1d_generator.cpp
    motion/lattice/trajectory_generation/piecewise_braking_trajectory_generator.cpp
    motion/lattice/trajectory_generation/trajectory_evaluator.cpp
    motion/lattice/trajectory_generation/trajectory_combiner.cpp
    motion/lattice/trajectory_generation/backup_trajectory_generator.cpp

    motion/lattice/lattice_planner.cpp
    planning.cpp
    main.cpp
    )

add_library(planning SHARED ${LATTICE_SRCS})
target_link_libraries(planning glog gflags protobuf)


