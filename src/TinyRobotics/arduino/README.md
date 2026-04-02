# Arduino Compatibility Stubs for Native Builds

This folder contains minimal Arduino compatibility headers and stubs to enable building and testing the TinyRobotics library on non-Arduino (native/desktop) platforms.

## Purpose

Arduino sketches and libraries depend on core Arduino classes and functions (e.g., `Serial`, `Print`, `Stream`, `millis()`, `pinMode`, etc.). These are not available on desktop systems. The files in this directory provide stub implementations so that code can be compiled and tested natively, including with CMake and standard C++ compilers.

## Key Files

- `Arduino.h`: Stubs for Arduino functions, macros, and types.
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

## Maintainers

If you add new Arduino API stubs, please document them here.
