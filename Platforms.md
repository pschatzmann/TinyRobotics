## Supported Platforms

### Arduino

This library should be working on all Arduino platforms that provide "enough" RAM and support the C++ STL:

- ESP32
- ESP8266
- RP2040
- Arduino Zephyr cores
- Arduino MKR boards
- etc

However, it might happen that motor libraries are not supported. In this case you can deactivate them in utils/Config.h or by using one of the following #defines before including the project:

- #define USE_FASTACCEL_STEPPER false
- #define USE_EXTERNAL_MOTOR_LIBRARIES false
- #define USE_SERVO_LIBRARY false

### Others

You can compile this project on the Desktop or for other microcontroller frameworks e.g. ESP32 IDF, Zephyr etc using CMAKE.