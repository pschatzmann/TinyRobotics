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

# Find installed libgpiod 
find_library(GPIOD_LIB NAMES gpiod gpiod2)
if(GPIOD_LIB)
  if(GPIOD_LIB MATCHES "gpiod2")
    add_compile_options(-DLIBGPIOD_V2)
  endif()
  message(STATUS "Found ${GPIOD_NAME}: ${GPIOD_LIB}")
else()
  message(WARNING "libgpiod not found - Raspberry Pi GPIO functionality may fail at link time.")
endif()

# Link arduino_emulator explicitly to the found lib if present
if(GPIOD_LIB)
  target_link_libraries(arduino_emulator PUBLIC ${GPIOD_LIB})
endif()

target_link_libraries(TinyRobotics INTERFACE arduino_emulator)
target_compile_options(TinyRobotics INTERFACE -DUSE_RPI=true -DLIBGPIOD_V2)


add_subdirectory(Mavlink-receive)
