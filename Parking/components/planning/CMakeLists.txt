cmake_minimum_required(VERSION 3.5)

project(planning_main CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(lcm REQUIRED)
find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ../
    ../lcmtype
    ${EIGEN3_INCLUDE_DIR}
    ${lcm_INCLUDE_DIR}
    )
link_directories(
    usr/local/lib
    )

set(PLANNING_SRCS
# common
    ../common/gflags_common.cpp
    # vehicle_param_handle
    ../common/vehicle_param_handle/vehicle_param_handle.cpp
    # visual_handle
    ../common/visual_handle/visual_handle.cpp
    # util
    ../common/util/file.cpp
    # math
    ../common/math/vec2d.cpp
    ../common/math/aabox2d.cpp
    ../common/math/line_segment2d.cpp
    ../common/math/math_utils.cpp
    ../common/math/angle.cpp
    ../common/math/sin_table.cpp
    ../common/math/qp_solver/active_set_qp_solver.cpp
    ../common/math/qp_solver/qp_solver.cpp
    ../common/math/qp_solver/qp_solver_gflags.cpp
    ../common/math/linear_interpolation.cpp
    # adapter
    ../common/adapters/adapter.h
    ../common/adapters/adapter_manager.cpp
    ../common/adapters/message_adapter.h
    ../common/adapters/proto/adapter_config.pb.cc
    # proto
    ../common/proto/vehicle_state.pb.cc
    ../common/proto/vehicle_param.pb.cc
    ../common/proto/pnc_point.pb.cc
# map
    # common
    ../map/common/gflags_map.cpp
    # map_path
    ../map/map_path/map_path.cpp
    # road_map
    ../map/road_map/road_map.cpp
    # pnc_path
    ../map/pnc_path/pnc_path.cpp
    # proto
    ../map/proto/created_map.pb.cc
    ../map/proto/pnc_path_config.pb.cc

# perception
    ../perception/proto/ogm_config.pb.cc
    ../perception/common/gflags_perception.cpp

# planning
    # common
    ../planning/common/gflags_planning.cpp
    ../planning/common/path/discretized_path.cpp
    ../planning/common/path/path_data.cpp
    ../planning/common/speed/speed_data.cpp
    ../planning/common/trajectory/discretized_trajectory.cpp
    ../planning/common/trajectory/publishable_trajectory.cpp
    ../planning/common/trajectory/trajectory_stitcher.cpp
    # reference_line
    ../planning/reference_line/reference_line.cpp
    ../planning/reference_line/reference_point.cpp
    ../planning/reference_line/reference_line_provider.cpp
    ../planning/reference_line/qp_spline_reference_line_smoother.cpp
    # math
    ../planning/math/polynomial_xd.cpp
    ../planning/math/smoothing_spline/affine_constraint.cpp
    ../planning/math/smoothing_spline/spline_2d.cpp
    ../planning/math/smoothing_spline/spline_2d_seg.cpp
    ../planning/math/smoothing_spline/spline_2d_constraint.cpp
    ../planning/math/smoothing_spline/spline_2d_kernel.cpp
    ../planning/math/smoothing_spline/spline_seg_kernel.cpp
    ../planning/math/smoothing_spline/spline_2d_solver.cpp
    ../planning/math/curve_math.cpp
    ../planning/math/curve1d/quartic_polynomial_curve1d.cpp
    ../planning/math/curve1d/quintic_polynomial_curve1d.cpp

    # proto
    ../planning/proto/fp_path_plan_config.pb.cc
    ../planning/proto/reference_line_provider_config.pb.cc
    ../planning/proto/poly_speed_plan_config.pb.cc
    ../planning/proto/planning_config.pb.cc
    # motion
      # fp_path_plan
    ../planning/motion/fp_path_paln/parallel_graph.cpp
    ../planning/motion/fp_path_paln/fp_path_planner.cpp
      # poly_speed_plan
    ../planning/motion/poly_speed_plan/poly_speed_planner.cpp
    ../planning/motion/poly_speed_plan/poly_st_graph.cpp
    ../planning/motion/poly_speed_plan/feasible_region.cpp
    ../planning/motion/poly_speed_plan/speed_profile.cpp

    )

add_executable(planning_main
    main.cpp
    planning.cpp
    ${PLANNING_SRCS}
    )

target_link_libraries(planning_main
    ${PROTOBUF_LIBRARIES}
    absl::strings
    lcm
    glog
    gflags
    pthread
    qpOASES
    opencv_imgproc
    opencv_highgui
    )

# exe out dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../../bin)
set_target_properties(planning_main PROPERTIES DEBUG_POSTFIX "_d")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin)



