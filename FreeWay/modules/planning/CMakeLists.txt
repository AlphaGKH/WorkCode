project(planning)

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})


file (GLOB PLANNING_PROTO
     ${dharma_SOURCE_DIR}/modules/*/proto/*.proto
)

PROTOC_CPP(PB_SRCS PB_HDRS PROTO_ROOT_PATH ${dharma_SOURCE_DIR} ${PLANNING_PROTO})

# planning_source
file(GLOB PLANNING_SRCS
    ./planning.cpp
    common/*.cpp
    common/*/*.cpp
    constraint_checker/*.cpp
    frame/*.cpp
    math/*/*.cpp
    motion/lattice/*/*.cpp
    obstacle/*.cpp
    planner/*.cpp
    reference_line/*.cpp
    speed/*.cpp
    trajectory/*.cpp

    ../common/*/*.cpp

    ../map/*/*.cpp
)

add_library(planning ${PLANNING_SRCS} ${PB_SRCS})
target_link_libraries(planning
    libabsl_strings.a
    pthread
    spider 
    gflags)
