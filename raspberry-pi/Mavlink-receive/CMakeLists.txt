# CMakeLists.txt for Mavlink-receive subdirectory

include(FetchContent)
FetchContent_Declare(
  ArduinoMavlinkDrone
  GIT_REPOSITORY https://github.com/pschatzmann/ArduinoMavlinkDrone.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(ArduinoMavlinkDrone)

# Add Arduino-Emulator include directory for WiFiUdp.h
include(FetchContent)
FetchContent_GetProperties(arduinoemulator)

set_source_files_properties(Mavlink-receive.ino PROPERTIES LANGUAGE CXX)
add_library(MavlinkReceiveLib STATIC Mavlink-receive.ino)

target_include_directories(MavlinkReceiveLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${arduinomavlinkdrone_SOURCE_DIR}/src
    ${arduinoemulator_SOURCE_DIR}/ArduinoCore-Linux/cores/arduino
    ${arduinoemulator_SOURCE_DIR}/ArduinoCore-API
)

target_compile_options(MavlinkReceiveLib PRIVATE -x c++ -DUSE_EXTERNAL_MOTOR_LIBRARIES=false -DUSE_TR_NAMESPACE=true)
target_link_libraries(MavlinkReceiveLib ArduinoEmulator TinyRobotics ArduinoMavlinkDrone)
