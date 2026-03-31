#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace tinyrobotics {

/**
 * @class LoggerClass
 * @ingroup utils
 * @brief Simple, cross-platform logger for Arduino and C++ environments.
 *
 * This class provides a lightweight logging facility with support for log levels (ERROR, WARN, INFO, DEBUG),
 * printf-style formatting, and platform-adaptive output (Serial for Arduino, printf for standard C++). It is
 * designed for embedded and robotics applications where monitoring, debugging, and event tracking are essential.
 *
 * ## Features
 * - Four log levels: ERROR, WARN, INFO, DEBUG
 * - Configurable minimum log level (messages below this are suppressed)
 * - printf-style formatting for flexible message construction
 * - Platform-adaptive output: uses Serial on Arduino, printf elsewhere
 * - Thread-safe for most embedded use cases (no dynamic memory allocation)
 * - Global logger instance (`TRLogger`) for convenience
 * 
 *
 * ## Usage Example
 * @code
 *   TRLogger.setLevel(LoggerClass::DEBUG);
 *   TRLogger.info("System started, version: %d", version);
 *   TRLogger.error("Failed to open file: %s", filename);
 *   TRLogger.debug("x=%.2f, y=%.2f", x, y);
 * @endcode
 *
 * ## Methods
 * - setLevel(level): Set the minimum log level
 * - error(fmt, ...);
 * - warn(fmt, ...); 
 * - info(fmt, ...);
 * - debug(fmt, ...);
 *
 * ## Log Levels
 * - ERROR: Critical errors and failures
 * - WARN: Warnings and recoverable issues
 * - INFO: Informational messages and status updates
 * - DEBUG: Detailed debug output for development
 *
 * ## Platform Support
 * - On Arduino, output is sent to Serial (or a user-supplied Print object)
 * - On standard C++, output is sent to stdout via printf
 *
 * @author Phil Schatzmann

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


/**
 * @brief Global logger instance for convenience.
 *
 * Use `TRLogger` for simple, global logging throughout your application.
 * Example:
 * @code
 *   TRLogger.info("Hello, world!");
 * @endcode
 */
static LoggerClass TRLogger;

}  // namespace tinyrobotics