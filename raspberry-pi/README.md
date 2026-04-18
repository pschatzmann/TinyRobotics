# Raspberry Pi Support

We can use the Arduino Emulator to run TinyRobotics sketches on a Raspberry Pi.

## Features
- Run and test TinyRobotics code natively on Raspberry Pi (Linux)
- Use the [Arduino-Emulator](https://github.com/pschatzmann/Arduino-Emulator) to provide Arduino API compatibility
- Integrate with additional Linux/Raspberry Pi features (e.g., camera, networking)
- Example: Receive MAVLink messages and control a simulated Ackerman car

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
   cmake -DUSE_PI=ON ..

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
- [ArduinoMavlinkDrone](https://github.com/pschatzmann/ArduinoMavlinkDrone) (fetched automatically by CMake)
- [TinyRobotics](https://github.com/pschatzmann/TinyRobotics)

## Notes
- Some Arduino libraries or hardware-specific features may not be available or fully emulated.
- You can extend the emulator or add Linux-specific code for advanced integration.

## License
See the main repository for license information.

