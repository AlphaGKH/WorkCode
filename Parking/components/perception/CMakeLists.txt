cmake_minimum_required(VERSION 3.5)

project(perception_main CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(gflags REQUIRED)
find_package(lcm REQUIRED)
find_package(absl REQUIRED)

include_directories(
    ../
    ../lcmtype
    ${lcm_INCLUDE_DIR}
    )
link_directories(
    usr/local/lib
    )

set(PERCEPTION_SRCS
# common
    ../common/gflags_common.cpp
    # util
    ../common/util/file.cpp
    # adapter
    ../common/adapters/adapter.h
    ../common/adapters/adapter_manager.cpp
    ../common/adapters/message_adapter.h
    ../common/adapters/proto/adapter_config.pb.cc
    # math
    ../common/math/vec2d.cpp
    ../common/math/math_utils.cpp
    # proto
    ../common/proto/vehicle_param.pb.cc
    # vehicle_param_handle
    ../common/vehicle_param_handle/vehicle_param_handle.cpp

# perception
    # common
    ../perception/common/gflags_perception.cpp
    # proto
    ../perception/proto/ogm_config.pb.cc
    ../perception/proto/perception_config.pb.cc

    # ogm_perception
    ../perception/ogm_perception/ogm_perception.cpp
    )

add_executable(perception_main
    main.cpp
    perception.cpp
    ${PERCEPTION_SRCS}
    )

target_link_libraries(perception_main
    ${PROTOBUF_LIBRARIES}
    absl::strings
    lcm
    glog
    gflags
    pthread
    )

# exe out dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../../bin)

#if(${CMAKE_BUILD_TYPE} MATCHES "DEBUG")
    set_target_properties(perception_main PROPERTIES DEBUG_POSTFIX "_d")
#endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin)



