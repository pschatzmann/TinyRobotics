# CMake configuration for TinyRobotics tests

cmake_minimum_required(VERSION 3.14)

project(TinyRoboticsTests LANGUAGES CXX)

# Add subdirectories for each test
add_subdirectory(circle)
add_subdirectory(control-simple)
add_subdirectory(control)

