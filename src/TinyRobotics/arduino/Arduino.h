#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include "Print.h"
#include "Stream.h"
#include "TinyRobotics/utils/Config.h"

#ifndef ARDUINO
#define ARDUINO 1
#endif

// Pin modes and digital values
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT
#define INPUT 0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif

namespace tinyrobotics_arduino  {

// Timing
inline unsigned long millis() {
  static auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
      .count();
}

// Pin functions (no-op for native)
#if USE_DUMMY_PIN_FUNCTIONS
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int analogRead(int) { return 0; }
inline void analogWrite(int, int) {}
#else
void pinMode(int, int);
void digitalWrite(int, int);
int analogRead(int);
void analogWrite(int, int);
#endif

// Math helpers
#ifndef constrain
template <typename T, typename L, typename H>
constexpr T constrain(T amt, L low, H high) {
  return std::min(std::max(amt, static_cast<T>(low)), static_cast<T>(high));
}
#endif
#ifndef map
#define map(x, in_min, in_max, out_min, out_max)                        \
  (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + \
   (out_min))
#endif

// Serial stub
struct HardwareSerial : public Stream {
  void begin(unsigned long) {}
   size_t write(uint8_t ch) override {
    std::cout << static_cast<char>(ch);
    return 1;
  }
};

static HardwareSerial Serial;

}  // namespace tinyrobotics_arduino

void setup();
void loop();

int main() {
  setup();
  while (true) {
    loop();
  }
}

