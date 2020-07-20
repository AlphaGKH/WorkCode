project(spider LANGUAGES CXX)

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})

# spider_source
file(GLOB SPIDER_SRCS
    common/*.cpp
    time/*.cpp
)

#build spider library
add_library(spider SHARED ${SPIDER_SRCS})
target_link_libraries(spider glog protobuf)


