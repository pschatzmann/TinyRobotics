## Supported Platforms

### Arduino

This library should be working on all Arduino platforms that provide "enough" RAM and support the __C++ STL__:

- __ESP32__
- __ESP8266__
- __RP2040__
- Arduino Zephyr cores
- Arduino MKR boards
- etc

However, it might happen that motor libraries are not supported. In this case you can deactivate them in __utils/Config.h__ or by using one of the following #defines before including the project:

- #define __USE_EXTERNAL_MOTOR_LIBRARIES__ false
- #define __USE_FASTACCEL_STEPPER__ false
- #define __USE_SERVO_LIBRARY__ false

This library uses the tinyrobotics namespace. If you want to access the functionality of this library outside of Arduino w/o specifiying the namespace you can use the following define:

- #define __USE_TR_NAMESPACE__: if defined we do not need to define the namespace in the Arduino Sketch.


## Other Platforms (using cmake)

You can compile this project on the __desktop__, __single-board computers__ (e.g. Raspbery PI) or for other __microcontroller frameworks__ e.g. ESP32 IDF, Zephyr, STM32 Cube IDC, etc using CMAKE.

### Internal Emulator

This library contains a simple Arduino Stream/Print emulattion that can be used to build this library locally with the help of cmake.

Further info can be found in the [TinyRobotics/arduino](src/TinyRobotics/arduino/README.md) folder

### External Arduino Emulator

To have access to the complete Arduino functionality you can use my [Arduino-Emulator](https://github.com/pschatzmann/Arduino-Emulator) that is supporting Linux, macOS and Windows.

Further info on how to use this functionality (on a Raspberry PI) can be found in the [raspberry-pi](raspberry-pi/README.md) folder.