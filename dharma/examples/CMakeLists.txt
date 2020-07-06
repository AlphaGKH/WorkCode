project(spider_example)

include_directories(${dharma_SOURCE_DIR})
include_directories(${dharma_BINARY_DIR})

add_compile_options(-Og)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/examples.proto)

# talker_pb
add_executable(talker_pb talker_pb.cpp ${PROTO_SRCS})
target_link_libraries(talker_pb spider)

# listener_pb
add_executable(listener_pb listener_pb.cpp ${PROTO_SRCS})
target_link_libraries(listener_pb spider)

## talker_pb
#add_executable(talker_lcm talker_lcm.cpp lcm/DemoLCM.hpp)
#target_link_libraries(talker_lcm spider)

## listener_pb
#add_executable(listener_lcm listener_lcm.cpp lcm/DemoLCM.hpp)
#target_link_libraries(listener_lcm spider)

# example_component
add_executable(example_component example_component/example_component.cpp ${PROTO_SRCS})
target_link_libraries(example_component spider)














