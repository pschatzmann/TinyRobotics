#pragma once
#include <vector>

#include "Print.h"

namespace tinyrobotics {

/**
 * @brief Utility class for writing output to multiple Print streams
 * simultaneously.
 *
 * MultiOutput allows you to broadcast print output to several Print targets
 * (e.g., Serial, file, network) at once. It implements the Print interface, so
 * it can be used anywhere a Print object is expected.
 *
 * Example usage:
 * @code
 *   MultiOutput multi;
 *   multi.addOutput(Serial);
 *   multi.addOutput(myFile);
 *   multi.println("Hello, world!"); // Prints to both Serial and file
 * @endcode
 *
 * This is useful for logging, debugging, or mirroring output to multiple
 * destinations in embedded systems.
 */
class MultiOutput : public Print {
 public:
  /// Defines a MultiOutput with no final output: Define your outputs with add()
  MultiOutput() = default;

  /**
   * @brief Construct MultiOutput with a variable number of Print objects.
   *
   * Example usage:
   * @code
   *   MultiOutput multi(Serial, myFile);
   * @endcode
   *
   * @param outputs Variable number of Print references.
   */
  template <typename... Prints>
  MultiOutput(Prints&... outputs) {
    addOutputs(outputs...);
  }

  virtual ~MultiOutput() { clear(); }

  void add(Print& print) { vector.push_back(&print); }
  void flush() {
    for (int j = 0; j < vector.size(); j++) {
      vector[j]->flush();
    }
  }

  size_t write(const uint8_t* data, size_t len) override {
    for (auto& out : vector) {
      int open = len;
      int start = 0;
      // create copy of data to avoid that one output changes the data for the
      // other outputs
      uint8_t copy[len];
      memcpy(copy, data, len);
      while (open > 0) {
        int written = out->write(copy + start, open);
        open -= written;
        start += written;
      }
    }
    return len;
  }

  size_t write(uint8_t ch) override {
    for (int j = 0; j < vector.size(); j++) {
      int open = 1;
      while (open > 0) {
        open -= vector[j]->write(ch);
      }
    }
    return 1;
  }

  /// Removes all output components
  void clear() {
    vector.clear();
  }

 protected:
  std::vector<Print*> vector;

  /// support for Pipleline
  void setOutput(Print& out) { add(out); }

  // Helper to add multiple outputs
  void addOutputs() {}
  template <typename First, typename... Rest>
  void addOutputs(First& first, Rest&... rest) {
    add(first);
    addOutputs(rest...);
  }
};

}  // namespace tinyrobotics
