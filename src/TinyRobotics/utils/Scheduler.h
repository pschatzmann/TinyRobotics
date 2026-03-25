#pragma once
#include <cstdint>
#ifdef ARDUINO
#include "Arduino.h"
#else
#include <chrono>
#endif

namespace tinyrobotics {


/**
 * @brief A simple scheduler class that allows you to schedule a callback
 * function to be called at a specified interval (repeatMs). The scheduler uses
 * the Arduino millis() function to track time and determine when to call the
 * callback. You can start the scheduler with the begin() method, which takes
 * the repeat interval and the callback function as parameters. The end() method
 * can be used to stop the scheduler. The run() method should be called
 * regularly (e.g., in the main loop) to check if it's time to execute the
 * callback. This class is useful for tasks that need to be performed
 * periodically without blocking the main loop, such as sensor readings, status
 * updates, or any recurring task
 *
 */
class Scheduler {
 public:
  Scheduler() = default;

  void begin(uint16_t repeatMs, void (*cb)(void*)) {
    repeatMs_ = repeatMs;
    start_time = millis() + repeatMs;
    setCallback(cb);
    is_active_ = true;
  }

  void end() { is_active_ = false; }

  void setCallback(void(*cb)(void*)) {
    // Store the callback function pointer for later use
    callback = cb;
  }

  void run() {
    if (is_active_ && callback && millis() >= start_time) {
      // Call the callback function
      callback(nullptr);
      start_time += repeatMs_;  // Schedule the next execution
    }
  }

#ifndef ARDUINO
  unsigned long millis() {
    // Implement a simple millis() function for non-Arduino environments
    static auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                 start_time)
        .count();
  }
#endif

 protected:
  unsigned long start_time = 0;
  bool is_active_ = false;
  void (*callback)(void*) = nullptr;
  uint16_t repeatMs_;
};

} // namespace tinyrobotics