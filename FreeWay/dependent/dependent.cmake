# protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
link_directories(${PROTOBUF_LIB_DIR})

find_package(absl REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# function for compiling proto file to *.pb.cc and *.pb.h
function(PROTOC_CPP SRCS HDRS)
    cmake_parse_arguments(protoc_cpp "" "PROTO_ROOT_PATH" "" ${ARGN})

    if(NOT protoc_cpp_PROTO_ROOT_PATH)
        set(protoc_cpp_PROTO_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()

    set(_proto_files "${protoc_cpp_UNPARSED_ARGUMENTS}") # 获取．proto文件

    if(NOT _proto_files)
        message(SEND_ERROR "Error: PROTOC_CPP() called without any proto files")
        return()
    endif()

    set(protoc_cpp_PROTO_DIST_DIR ${protoc_cpp_PROTO_ROOT_PATH}/build/)
    set(protoc_cpp_GENERATE_EXTENSIONS .pb.h .pb.cc)
    
    set(_generated_srcs_all)
    foreach(FIL ${_proto_files})
        get_filename_component(FIL_WE ${FIL} NAME_WE)

        string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
        string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})

        string(REGEX REPLACE "${protoc_cpp_PROTO_ROOT_PATH}" "${protoc_cpp_PROTO_DIST_DIR}" OUT_PATH ${FILE_PATH})

        set(_generated_srcs)
        foreach(_ext ${protoc_cpp_GENERATE_EXTENSIONS})
            list(APPEND _generated_srcs "${OUT_PATH}${FIL_WE}${_ext}")
        endforeach()

        list(APPEND _generated_srcs_all ${_generated_srcs})

        add_custom_command(
            OUTPUT ${_generated_srcs}
            COMMAND protobuf::protoc ${FIL} --cpp_out ${protoc_cpp_PROTO_DIST_DIR} -I${protoc_cpp_PROTO_ROOT_PATH}
            DEPENDS protobuf::protoc ${_proto_files}
        )
    endforeach(FIL ${_proto_files})

    set_source_files_properties(${_generated_srcs_all} PROPERTIES GENERATED TRUE)

    set(${SRCS})
    set(${HDRS})

    foreach(_file ${_generated_srcs_all})
        if(_file MATCHES "cc$")
            list(APPEND ${SRCS} ${_file})
        else()
            list(APPEND ${HDRS} ${_file})
        endif()
    endforeach()

    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)

endfunction()
