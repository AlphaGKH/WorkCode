cmake_minimum_required(VERSION 3.5)

project(create_road_map LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(lcm REQUIRED)

include_directories(
    ../../components/
    ../../components/lcmtype/
    ${lcm_INCLUDE_DIR}
    ./
    )
link_directories(
    usr/local/lib
    )



set(SRCS
# creat_road_map
    generate_road_map/generate_road_map.cpp
    gflags/create_road_map_gflags.cpp
# components
    ../../components/common/util/file.cpp
    ../../components/map/proto/created_map.pb.cc
    ../../components/common/adapters/adapter_manager.cpp
    ../../components/common/adapters/proto/adapter_config.pb.cc
    )

add_executable(create_road_map
    main.cpp
    ${SRCS}
    )

target_link_libraries(create_road_map
    ${PROTOBUF_LIBRARIES}
    glog
    gflags
    lcm
    pthread
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../bin)
set_target_properties(create_road_map PROPERTIES DEBUG_POSTFIX "_d")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)
