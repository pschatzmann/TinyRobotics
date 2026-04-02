#pragma once

namespace tinyrobotics_arduino  {

/**
 * @brief Arduino-compatible Print base class stub for native builds.
 *
 * This class provides a minimal interface compatible with the Arduino Print class,
 * allowing code that uses Print (and derived classes like Serial, Stream, etc.)
 * to compile and run on non-Arduino (desktop/native) platforms.
 *
 * Methods such as write(), print(), and println() mimic the Arduino API, enabling
 * easy porting and testing of Arduino libraries and sketches in a standard C++ environment.
 *
 * To use, derive from Print and implement the write(uint8_t) method to define how
 * output is handled (e.g., to std::cout, a file, or a buffer).
 */

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(uint8_t* buf, size_t, size_t len) {
    size_t total = 0;
    for (size_t i = 0; i < len; ++i) {
      total += write(buf[i]);
    }
    return total;
  }
  virtual size_t write(const uint8_t* buf, size_t len) {
    return write((uint8_t*)buf, 0, len);
  }

  size_t print(const char* str) {
    size_t len = 0;
    while (str[len] != '\0') {
      write(str[len]);
      len++;
    }
    return len;
  }
  size_t println(const char* str) {
    size_t len = print(str);
    write('\n');
    return len + 1;
  }
  size_t print(float number, int precision = 2) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f", precision, number);
    return print(buffer);
  }
  size_t println(int value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    return println(buf);
  }
  size_t println(float value) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%f", value);
    return println(buf);
  }
  size_t print(int value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    return print(buf);
  }
  void flush() {}
  int availableForWrie() { return 256; }
};

}  // namespace tinyrobotics_arduino

#ifdef USE_ARDUINO_EMULATION
using namespace tinyrobotics_arduino;
#endif