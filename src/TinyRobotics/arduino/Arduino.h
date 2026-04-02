#pragma once
#ifdef USE_TR_ARDUINO_EMULATION
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include "TinyRobotics/utils/Config.h"
#include "Stream.h"
#include "TinyRobotics/utils/LoggerClass.h"


namespace tinyrobotics_arduino {
enum PinMode { INPUT = 0, OUTPUT = 1 };
enum DigitalValue { LOW = 0, HIGH = 1 };

// Timing
#if USE_MILLIS_CHRONO
inline unsigned long millis() {
  static auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
      .count();
}
#else
unsigned long millis();
#endif

// Pin functions (no-op for native)
#if USE_DUMMY_PIN_FUNCTIONS
inline void pinMode(int pin, int mode) {
  tinyrobotics::TRLogger.info("Emulator: pinMode(%d,%d)", pin, mode);
}
inline void digitalWrite(int pin, int value) {
  tinyrobotics::TRLogger.info("Emulator: digitalWrite(%d,%d)", pin, value);
}
inline void analogWrite(int pin, int value) {
  tinyrobotics::TRLogger.info("Emulator: analogWrite(%d,%d)", pin, value);
}
#else
void pinMode(int, int);
void digitalWrite(int, int);
void analogWrite(int, int);
#endif

// Math helper: constrains a value between a minimum and maximum
#ifndef constrain
template <typename T, typename L, typename H>
constexpr T constrain(T amt, L low, H high) {
  return std::min(std::max(amt, static_cast<T>(low)), static_cast<T>(high));
}
#endif

// Math helper: maps a value from one range to another
#ifndef map
template <typename T, typename U>
constexpr auto map(T x, U in_min, U in_max, U out_min, U out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif


#if USE_PRINT_CHAR
bool print_char(uint8_t ch) {
  std::cout << static_cast<char>(ch);
  return true;
}
#else
bool print_char(uint8_t);
#endif


/// Serial stub class
struct HardwareSerial1 : public Stream {
  void begin(unsigned long) {}
  int read() override { return -1; } 
  int peek() override { return -1; }
  int available() override { return 0; }
  size_t write(uint8_t ch) override {
    return print_char(ch) ? 1 : 0;
  }
};

static HardwareSerial1 Serial;

}  // namespace tinyrobotics_arduino

void setup();
void loop();

int main() {
  setup();
  while (true) {
    loop();
  }
}

#endif
