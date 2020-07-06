cmake_minimum_required(VERSION 3.12)

project(dharma LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
enable_language(ASM)

# cmake constant
include(GNUInstallDirs)

# third_libs
include(cmake/third_libs.cmake)

# output dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/)

# cyber
add_subdirectory(cyber)

# moduless

## planning
#add_subdirectory(modules/planning)
add_subdirectory(modules/planning)
