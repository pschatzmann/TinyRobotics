cmake_minimum_required(VERSION 3.10)

project(TinyRobotics LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE TINYROBOTICS_SRC
    src/TinyRobotics/*.h
    src/TinyRobotics/*.cpp
)

add_library(TinyRobotics INTERFACE)
target_include_directories(TinyRobotics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Optionally, if you want to build examples:
option(BUILD_EXAMPLES "Build example sketches" ON)
if(BUILD_EXAMPLES)
endif()
