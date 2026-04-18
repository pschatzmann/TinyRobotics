# CMakeLists.txt for Mavlink-receive subdirectory

arduino_library(ArduinoMavlinkDrone https://github.com/pschatzmann/ArduinoMavlinkDrone.git TAG main)
arduino_sketch(MavlinkReceive Mavlink-receive.ino LIBRARIES arduino_emulator TinyRobotics ArduinoMavlinkDrone)

target_compile_options(MavlinkReceive PUBLIC  -DUSE_RPI=true -DUSE_TR_NAMESPACE=true)

