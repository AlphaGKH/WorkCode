project(cyber LANGUAGES CXX)

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})

# file
set(FILE_OP_SRCS
    common/file.cpp
    )


add_library(file_op SHARED ${FILE_OP_SRCS})
target_link_libraries(file_op glog)


