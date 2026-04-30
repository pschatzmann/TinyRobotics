# CMake configuration for control-simple.cpp test

cmake_minimum_required(VERSION 3.14)

# Project name
project(control-simple)

# Treat control-simple.ino as a C++ file
set_source_files_properties(control-simple.ino PROPERTIES LANGUAGE CXX)
set_source_files_properties(control-simple.ino PROPERTIES COMPILE_FLAGS "-x c++")
add_executable(control-simple control-simple.ino)
# configure your custom options:
target_compile_options(control-simple PRIVATE -DUSE_TR_ARDUINO_EMULATION -DUSE_EXTERNAL_MOTOR_LIBRARIES=0 -DUSE_DUMMY_PIN_FUNCTIONS=1 -DUSE_MILLIS_CHRONO=1 -DUSE_PRINT_CHAR=1 -DUSE_TR_NAMESPACE)
target_link_libraries(control-simple TinyRobotics)
