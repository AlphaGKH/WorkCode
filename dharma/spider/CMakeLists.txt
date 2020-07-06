project(spider LANGUAGES CXX)

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})

file(GLOB SPIDER_PROTO proto/*.proto)

PROTOBUF_GENERATE_CPP(SPIDER_PROTO_SRCS SPIDER_PROTO_HDRS ${SPIDER_PROTO})

# spider_source
file(GLOB SPIDER_SRCS
    *.cpp
    common/*.cpp
    spinner/*.cpp
    node/*.cpp
    time/*.cpp
)

#build spider library
add_library(spider SHARED ${SPIDER_SRCS} ${SPIDER_PROTO_SRCS})
target_link_libraries(spider lcm glog protobuf atomic pthread)


