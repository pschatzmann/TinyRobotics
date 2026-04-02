# CMake configuration for control.cpp test

cmake_minimum_required(VERSION 3.5)

# Project name
project(control_test)

# Treat control.ino as a C++ file
set_source_files_properties(control.ino PROPERTIES LANGUAGE CXX)
set_source_files_properties(control.ino PROPERTIES COMPILE_FLAGS "-x c++")
add_executable(control_test control.ino)
target_compile_options(control_test PRIVATE -DUSE_ARDUINO_EMULATION -DUSE_EXTERNAL_MOTOR_LIBRARIES=0 -DUSE_DUMMY_PIN_FUNCTIONS=1 -DUSE_MILLIS_CHRONO=1 -DUSE_PRINT_CHAR=1)
target_link_libraries(control_test TinyRobotics)
