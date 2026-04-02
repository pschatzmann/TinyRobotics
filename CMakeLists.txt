cmake_minimum_required(VERSION 3.10)

project(TinyRobotics LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(TinyRobotics INTERFACE)
target_include_directories(TinyRobotics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/TinyRobotics/arduino)
target_compile_options(TinyRobotics INTERFACE -DUSE_ARDUINO_EMULATION -DUSE_EXTERNAL_MOTOR_LIBRARIES=0 -DUSE_DUMMY_PIN_FUNCTIONS=1 -DUSE_MILLIS_CHRONO=1 -DUSE_PRINT_CHAR=1)

# Optionally, if you want to build examples:
option(BUILD_TESTS "Build Tests" ON)
if(BUILD_TESTS)
add_subdirectory(tests)
endif()
