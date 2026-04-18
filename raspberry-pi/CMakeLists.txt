# CMakeLists.txt for Raspberry Pi subdirectory

cmake_minimum_required(VERSION 3.14)

# Enable FetchContent module
include(FetchContent)

# Fetch Arduino-Emulator from GitHub
FetchContent_Declare(
  ArduinoEmulator
  GIT_REPOSITORY https://github.com/pschatzmann/Arduino-Emulator.git
  GIT_TAG        main
  CMAKE_ARGS -DUSE_RPI=ON
)
FetchContent_MakeAvailable(ArduinoEmulator)

add_subdirectory(Mavlink-receive)
