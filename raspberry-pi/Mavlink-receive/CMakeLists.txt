

# CMakeLists.txt for Mavlink-receive subdirectory
arduino_library(ArduinoMavlinkDrone https://github.com/pschatzmann/ArduinoMavlinkDrone.git TAG main)
arduino_sketch(MavlinkReceive Mavlink-receive.ino LIBRARIES  TinyRobotics ArduinoMavlinkDrone arduino_emulator)

# Comple options for TinyRobotics (e.g., to enable namespace usage)
target_compile_options(MavlinkReceive PUBLIC -DUSE_TR_NAMESPACE=true)

