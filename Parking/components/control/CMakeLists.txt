cmake_minimum_required(VERSION 3.5)

project(control_main C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LANGUAGE C)

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

set(MPC_SRCS
    #MPC
    ../control/MPC/amd_1.c
    ../control/MPC/amd_2.c
    ../control/MPC/amd_aat.c
    ../control/MPC/amd_control.c
    ../control/MPC/amd_defaults.c
    ../control/MPC/amd_info.c
    ../control/MPC/amd_order.c
    ../control/MPC/amd_postorder.c
    ../control/MPC/amd_post_tree.c
    ../control/MPC/amd_preprocess.c
    ../control/MPC/amd_valid.c
    ../control/MPC/auxil.c
    ../control/MPC/cs.c
    ../control/MPC/ctrlc.c
    ../control/MPC/kkt.c
    ../control/MPC/lib_handler.c
    ../control/MPC/lin_alg.c
    ../control/MPC/lin_sys.c
    ../control/MPC/memorypool.c
    ../control/MPC/MPC_light_matrix.c
    ../control/MPC/osqp.c
    ../control/MPC/pardiso_interface.c
    ../control/MPC/pardiso_loader.c
    ../control/MPC/polish.c
    ../control/MPC/proj.c
    ../control/MPC/qdldl.c
    ../control/MPC/qdldl_interface.c
    ../control/MPC/scaling.c
    ../control/MPC/SuiteSparse_config.c
    ../control/MPC/util.c
    ../control/MPC/MPC_Controller.c
    )

set(CONTROL_SRCS
# common
    ../common/gflags_common.cpp
    # util
    ../common/util/file.cpp
    # adapter
    ../common/adapters/adapter.h
    ../common/adapters/adapter_manager.cpp
    ../common/adapters/message_adapter.h
    ../common/adapters/proto/adapter_config.pb.cc
    # proto
    ../common/proto/vehicle_param.pb.cc
    # vehicle_param_handle
    ../common/vehicle_param_handle/vehicle_param_handle.cpp

# control
    # common
    ../control/common/gflags_control.cpp
    # proto
    ../control/proto/control_config.pb.cc
    # mpc
    ${MPC_SRCS}


    )

add_executable(control_main
    main.cpp
    control.cpp
    ${CONTROL_SRCS}
    )

target_link_libraries(control_main
    ${PROTOBUF_LIBRARIES}
    absl::strings
    lcm
    glog
    gflags
    pthread
    dl
    )

# exe out dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../../bin)

#if(${CMAKE_BUILD_TYPE} MATCHES "DEBUG")
    set_target_properties(control_main PROPERTIES DEBUG_POSTFIX "_d")
#endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin)



