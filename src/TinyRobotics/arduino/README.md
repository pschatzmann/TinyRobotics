# TinyRobotics Internal Arduino Emulation Layer

This folder contains minimal Arduino compatibility headers and stubs to enable building and testing the TinyRobotics library on non-Arduino (native/desktop) platforms or other non Arduino based microcontroller platforms.

## Purpose

Arduino sketches and libraries depend on core Arduino classes and functions (e.g., `Serial`, `Print`, `Stream`, `millis()`, `pinMode`, etc.). These are not available on desktop systems. The files in this directory provide stub implementations so that code can be compiled and tested natively, including with CMake and standard C++ compilers.

## Key Files

- `Arduino.h`: Stubs for Arduino functions, macros, and types:

  - unsigned long millis()
  - void delay()
  - void pinMode(int, int)
  - void digitalWrite(int, int)
  - void analogWrite(int, int)
  - map(T x, U in_min, U in_max, U out_min, U out_max)
  - constrain(T amt, L low, H high)
  - void main()
  - void setup()
  - void loop()

- `Print.h`: Minimal base class for print-style output.
- `Stream.h`: Minimal base class for stream-style input/output.
- `SerialStub.h` (if present): Stub for the `Serial` object.

## Usage

These headers are automatically included when building for native/desktop targets. They are not used on real Arduino hardware.

You can extend these stubs to add more Arduino API coverage as needed for your tests.

## Example

```cpp
#include <TinyRobotics/arduino/Arduino.h>

Serial.write('A'); // Outputs 'A' to stdout in native builds
```

## Defines

The emulator can be configured with the following defines:
- USE_TR_ARDUINO_EMULATION: activate this functionality
- USE_MILLIS_CHRONO: if set to true we implement millis() with the help of the c++ chrono. If false you need to provide your own implementation
- USE_DUMMY_PIN_FUNCTIONS: if set to true we just log the method calls; set to false to provide your own implementations
- USE_PRINT_CHAR: if set to true we print to the console; if set to flase provide your own implementation for the ```bool print_char(uint8_t)``` method


## Remarks

- If you add new Arduino API stubs, please document them here.
- If you need a complete Arduino Emulation you can use the [Arduino Emulator](https://github.com/pschatzmann/Arduino-Emulator) project.
