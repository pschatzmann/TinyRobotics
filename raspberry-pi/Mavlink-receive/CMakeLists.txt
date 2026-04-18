# CMakeLists.txt for Mavlink-receive subdirectory

arduino_library(ArduinoMavlinkDrone https://github.com/pschatzmann/ArduinoMavlinkDrone.git TAG main)
arduino_sketch(MavlinkReceive Mavlink-receive.ino LIBRARIES arduino_emulator TinyRobotics ArduinoMavlinkDrone)

# Common compile options for Raspberry Pi (e.g., to enable specific features or optimizations)
compile_options(-DUSE_RPI=true -DLIBGPIOD_V2)

# Comple options for TinyRobotics (e.g., to enable namespace usage)
target_compile_options(MavlinkReceive PUBLIC -DUSE_TR_NAMESPACE=true)

