# CMake configuration for circle.cpp test

cmake_minimum_required(VERSION 3.5)

# Project name
project(circle)

# Treat circle.ino as a C++ file
set_source_files_properties(circle.ino PROPERTIES LANGUAGE CXX)
set_source_files_properties(circle.ino PROPERTIES COMPILE_FLAGS "-x c++")
add_executable(circle circle.ino)
# configure your custom options:
target_compile_options(circle PRIVATE -DUSE_TR_ARDUINO_EMULATION -DUSE_EXTERNAL_MOTOR_LIBRARIES=0 -DUSE_DUMMY_PIN_FUNCTIONS=1 -DUSE_MILLIS_CHRONO=1 -DUSE_PRINT_CHAR=1 -DUSE_TR_NAMESPACE)
target_link_libraries(circle TinyRobotics)
