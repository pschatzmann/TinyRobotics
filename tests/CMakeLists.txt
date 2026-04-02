# CMake configuration for TinyRobotics tests

cmake_minimum_required(VERSION 3.5)

project(TinyRoboticsTests)

# Add subdirectories for each test
add_subdirectory(circle)
add_subdirectory(control-simple)
add_subdirectory(control)

