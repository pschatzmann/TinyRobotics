#pragma once
#include <cstdint>
#ifdef ARDUINO
#include "Arduino.h"
#else
#include <chrono>
#endif

namespace tinyrobotics {

/**
 * @class Scheduler
 * @ingroup control
 * @brief Simple periodic task scheduler for embedded and Arduino environments.
 *
 * This class provides a lightweight, non-blocking scheduler for executing a
 * callback function at a fixed interval (in milliseconds). It is designed for
 * use in embedded systems and Arduino sketches, where you want to perform
 * periodic tasks (such as sensor readings, status updates, or control actions)
 * without blocking the main loop.
 *
 * ## Features
 * - Works with Arduino (uses millis()) and desktop (uses std::chrono)
 * - Non-blocking: call run() regularly to trigger the callback at the right
 * time
 * - Easy to start, stop, and change the callback
 * - Suitable for cooperative multitasking and periodic polling
 *
 * ## Usage Example
 * @code
 *   void myTask(void*) {}
 *   Scheduler sched;
 *   sched.begin(1000, myTask); // Call myTask every 1000 ms
 *   // In your main loop:
 *   while (true) {
 *     sched.run();
 *     // ... other code ...
 *   }
 * @endcode
 *
 * ## Methods
 * - begin(repeatMs, callback): Start the scheduler with interval and callback
 * - end(): Stop the scheduler
 * - setCallback(callback): Change the callback function
 * - run(): Check and execute the callback if interval elapsed
 *
 * ## Applications
 * - Periodic sensor polling
 * - Status LED blinking
 * - Timed control actions
 * - Cooperative multitasking in embedded systems
 *
 * @author Phil Schatzmann

 */

class Scheduler {
 public:
  Scheduler() = default;

  void begin(uint16_t repeatMs, void (*cb)(void*), void* ref = nullptr) {
    repeatMs_ = repeatMs;
    start_time = millis() + repeatMs;
    reference = ref;
    setCallback(cb);
    is_active_ = repeatMs > 0 && cb != nullptr;
  }

  void end() { is_active_ = false; }

  void setCallback(void (*cb)(void*)) {
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

  bool isActive() const { return is_active_; }

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
  void *reference = nullptr;
  uint16_t repeatMs_;
};

}  // namespace tinyrobotics