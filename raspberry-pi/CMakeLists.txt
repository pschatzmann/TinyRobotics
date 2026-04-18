# CMakeLists.txt for Raspberry Pi subdirectory

cmake_minimum_required(VERSION 3.14)

# Enable FetchContent module
include(FetchContent)

# Fetch Arduino-Emulator from GitHub
FetchContent_Declare(
  arduino_emulator
  GIT_REPOSITORY https://github.com/pschatzmann/Arduino-Emulator.git
  GIT_TAG        main
  CMAKE_ARGS -DUSE_RPI=ON
  GIT_CLONE_TIMEOUT 600  # Timeout in seconds (e.g., 5 minutes)
)
FetchContent_MakeAvailable(arduino_emulator)

add_subdirectory(Mavlink-receive)
