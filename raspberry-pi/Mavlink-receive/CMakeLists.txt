# CMakeLists.txt for Mavlink-receive subdirectory
# Try to find libgpiod and link it so shared libs that use it resolve at link time

arduino_library(ArduinoMavlinkDrone https://github.com/pschatzmann/ArduinoMavlinkDrone.git TAG main)
arduino_sketch(MavlinkReceive Mavlink-receive.ino LIBRARIES arduino_emulator TinyRobotics ArduinoMavlinkDrone ${GPIOD_LIB})

# Comple options for TinyRobotics (e.g., to enable namespace usage)
target_compile_options(MavlinkReceive PUBLIC -DUSE_TR_NAMESPACE=true)

