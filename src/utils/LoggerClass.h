#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace tinyrobotics {

/**
 * @brief   A simple logging class that provides methods for logging messages at
 * different levels (ERROR, WARN, INFO, DEBUG). The logger can be configured to
 * set a minimum log level, so that messages below that level will not be
 * printed. The log messages are formatted using printf-style formatting, and
 * the output includes the log level as a prefix. This class can be used for
 * debugging and monitoring the behavior of the system, allowing developers to
 * easily track important events and diagnose issues. The logger can be used in
 * both Arduino and standard C++ environments, with appropriate handling for
 * output (Serial for Arduino, printf for standard C++).
 *
 */
class LoggerClass {
 public:
  enum Level { ERROR = 0, WARN = 1, INFO = 2, DEBUG = 3 };

  LoggerClass(Level level = INFO) : minLevel(level) {}

#ifdef ARDUINO
  bool begin(Level level, Print& out = Serial) {
    setLevel(level);
    p_out = &out;
    return true;
  }
#else
  bool begin(Level level) { setLevel(level); }
#endif

  void setLevel(Level level) { minLevel = level; }

  void log(Level level, const char* fmt, ...) {
    if (level > minLevel) return;
    const char* levelStr = levelToString(level);
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
#ifdef ARDUINO
    Serial.print(levelStr);
    Serial.print(": ");
    Serial.println(buf);
#else
    printf("%s: %s\n", levelStr, buf);
#endif
  }

  void error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(ERROR, fmt, args);
    va_end(args);
  }
  void warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(WARN, fmt, args);
    va_end(args);
  }
  void info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(INFO, fmt, args);
    va_end(args);
  }
  void debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(DEBUG, fmt, args);
    va_end(args);
  }

 protected:
  Level minLevel;
#ifdef ARDUINO
  Print* p_out = &Serial;
#endif

  void vlog(Level level, const char* fmt, va_list args) {
    if (level > minLevel) return;
    const char* levelStr = levelToString(level);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
#ifdef ARDUINO
    p_out->print(levelStr);
    p_out->print(": ");
    p_out->println(buf);
#else
    printf("%s: %s\n", levelStr, buf);
#endif
  }

  const char* levelToString(Level level) const {
    switch (level) {
      case ERROR:
        return "ERROR";
      case WARN:
        return "WARN";
      case INFO:
        return "INFO";
      case DEBUG:
        return "DEBUG";
      default:
        return "LOG";
    }
  }
};

static LoggerClass TRLogger;

}  // namespace tinyrobotics