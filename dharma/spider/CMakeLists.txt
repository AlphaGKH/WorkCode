project(spider LANGUAGES CXX)

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})

file(GLOB SPIDER_PROTO proto/*.proto)

PROTOC_CPP(SPIDER_PROTO_SRCS SPIDER_PROTO_HDRS PROTO_ROOT_PATH ${dharma_SOURCE_DIR} ${SPIDER_PROTO})

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


