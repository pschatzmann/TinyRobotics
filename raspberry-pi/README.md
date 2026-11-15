# Raspberry Pi Support

We can use the Arduino Emulator to run TinyRobotics sketches on a Raspberry Pi.

## Features
- Run and test TinyRobotics code natively on Raspberry Pi (Linux)
- Use the [Arduino-Emulator](https://github.com/pschatzmann/Arduino-Emulator) to provide Arduino API compatibility
- Integrate with additional Linux/Raspberry Pi features (e.g., camera, networking)
- Example: Receive MAVLink messages and control a simulated Ackerman car
- Servo and stepper motor drivers use hardware PWM on pins 12, 13, 18 and 19. The stepper driver supports continuous speed only.

## Getting Started

### Prerequisites
- CMake >= 3.14
- g++ (Linux)
- git
- Raspberry Pi OS or any Linux distribution

### Build Instructions

1. Clone this repository:
   ```sh
   git clone https://github.com/pschatzmann/TinyRobotics.git
   cd TinyRobotics
   ```
2. Create a build directory and configure with CMake:
   ```sh
   mkdir build && cd build
   cmake -DBUILD_PI=ON ..

   ```
   To use a local copy of the Arduino-Emulator instead of fetching it from GitHub:
   ```sh
   cmake -DBUILD_PI=ON -DFETCHCONTENT_SOURCE_DIR_ARDUINO_EMULATOR=/path/to/Arduino-Emulator ..
   ```
3. Build the project:
   ```sh
   make
   ```

### Example: Mavlink-receive

The `raspberry-pi/Mavlink-receive` directory contains an example that:
- Connects to WiFi
- Starts a UDP endpoint for MAVLink
- Receives MAVLink messages and controls a simulated Ackerman car
- Optionally starts a camera stream using `rpicam-vid` and `v4l2rtspserver`


## Dependencies
- [Arduino-Emulator](https://github.com/pschatzmann/Arduino-Emulator) (fetched automatically by CMake)
- [ArduinoMavlink](https://github.com/pschatzmann/ArduinoMavlinkDrone) (repository ArduinoMavlinkDrone; fetched automatically by CMake; provides `Mavlink.h` and `Mavlink/SimpleUDP.h`)
- [TinyRobotics](https://github.com/pschatzmann/TinyRobotics)

## Notes
- Some Arduino libraries or hardware-specific features may not be available or fully emulated.
- You can extend the emulator or add Linux-specific code for advanced integration.

## License
See the main repository for license information.

