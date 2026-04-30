cmake_minimum_required(VERSION 3.14)

project(TinyRobotics LANGUAGES CXX)

# Ensure required static library variables are set (needed by some Arduino CMake toolchains)
if(NOT CMAKE_C_CREATE_STATIC_LIBRARY)
  set(CMAKE_C_CREATE_STATIC_LIBRARY "<CMAKE_AR> cr <TARGET> <LINK_FLAGS> <OBJECTS>")
endif()
if(NOT CMAKE_CXX_CREATE_STATIC_LIBRARY)
  set(CMAKE_CXX_CREATE_STATIC_LIBRARY "<CMAKE_AR> cr <TARGET> <LINK_FLAGS> <OBJECTS>")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(TinyRobotics INTERFACE)
target_include_directories(TinyRobotics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src )

# Optionally, if you want to build examples:
option(BUILD_PI "Build for Raspberry PI" OFF)
if(BUILD_PI)
set(USE_RPI ON CACHE BOOL "")
add_definitions(-DUSE_RPI=1)
add_subdirectory(raspberry-pi)
else()
target_include_directories(TinyRobotics INTERFACE  ${CMAKE_CURRENT_SOURCE_DIR}/src/TinyRobotics/arduino)
endif()

# Optionally, if you want to build examples:
option(BUILD_TESTS "Build Tests" ON)
if(BUILD_TESTS)
add_subdirectory(tests)
endif()
