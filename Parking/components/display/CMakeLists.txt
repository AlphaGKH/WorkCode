cmake_minimum_required(VERSION 3.5)

project(display_main CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gflags REQUIRED)
find_package(lcm REQUIRED)
find_package(absl REQUIRED)

include_directories(
    ../
    ../lcmtype
    ${OpenCV_INCLUDE_DIR}
    ${lcm_INCLUDE_DIR}
    )

link_directories(
    usr/local/lib
    )

set(DISPLAY_SRCS
# common
    ../common/gflags_common.cpp
    # vehicle_param_handle
    ../common/vehicle_param_handle/vehicle_param_handle.cpp
    # util
    ../common/util/file.cpp
    # math
    ../common/math/vec2d.cpp
    ../common/math/aabox2d.cpp
    ../common/math/line_segment2d.cpp
    ../common/math/math_utils.cpp
    # adapter
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
    # proto
    ../map/proto/created_map.pb.cc

# planning
    # reference_line
    ../planning/reference_line/reference_point.cpp

# display
    # common
    ../display/common/gflags_display.cpp
    # global_path
    ../display/global_path/global_path.cpp
    # local_view
    ../display/local_view/local_view.cpp
    # proto
    ../display/proto/display_config.pb.cc

    )

add_executable(display_main
    main.cpp
    display.cpp
    ${DISPLAY_SRCS}
    )

target_link_libraries(display_main
    ${PROTOBUF_LIBRARIES}
    absl::strings
    lcm
    glog
    gflags
    pthread
    opencv_imgproc
    opencv_highgui
    )

# exe out dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../../bin)
set_target_properties(display_main PROPERTIES DEBUG_POSTFIX "_d")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin)
