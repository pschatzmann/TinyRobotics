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

### Others

You can compile this project on the __desktop__, __single-board computers__ (e.g. Raspbery PI) or for other __microcontroller frameworks__ e.g. ESP32 IDF, Zephyr, STM32 Cube IDC, etc using CMAKE.