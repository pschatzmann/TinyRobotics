#pragma once
#include "Print.h"

namespace tinyrobotics_arduino  {

/**
 * @brief Arduino-compatible Stream base class stub for native builds.
 *
 * This class mimics the Arduino Stream class, providing a minimal interface for
 * input/output streams in a desktop/native environment. It derives from Print and
 * adds methods such as available(), read(), peek(), flush(), and readBytes().
 *
 * Use this as a base for implementing Serial, file, or memory streams when porting
 * Arduino code to standard C++ platforms.
 */

class Stream : public Print {
 public:
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual int peek() { return -1; }
  virtual void flush() {}
  // Arduino-style readBytes stub
  virtual size_t readBytes(char* buffer, size_t len) {
    for (size_t i = 0; i < len; ++i) buffer[i] = 0;
    return len;
  }
};

}  // namespace tinyrobotics_arduino
