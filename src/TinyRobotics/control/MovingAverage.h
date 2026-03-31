#pragma once
#include <list>

namespace tinyrobotics {


/**
 * @class MovingAverage
 * @ingroup control
 * @brief Computes the moving average of a sequence of values.
 *
 * This template class maintains a fixed-size window of the most recent values and efficiently computes
 * their arithmetic mean. It is useful for smoothing noisy sensor data, filtering out short-term fluctuations,
 * and providing a simple low-pass filter in embedded and robotics applications.
 *
 * ## Features
 * - Generic: supports any numeric type (float, int, double, etc.)
 * - Fixed window size, configurable at runtime
 * - Efficient update and calculation
 *
 * ## Usage Example
 * @code
 *   MovingAverage<float> avg(5); // 5-sample moving average
 *   avg.addMeasurement(1.0f);
 *   avg.addMeasurement(2.0f);
 *   float mean = avg.calculate();
 * @endcode
 *
 * @tparam N Numeric type of the values (e.g., float, int)
 *
 * @author Phil Schatzmann

 */

template <class N = float>
class MovingAverage {
 public:
  MovingAverage(size_t size) {
    setSize(size);
  }

  void addMeasurement(N value) {
    if (this->values.size() == this->size) {
      this->values.pop_front();
    }
    this->values.push_back(value);
  }

  float calculate() {
    float sum = 0;
    for (int i = 0; i < this->values.size(); i++) {
      sum += this->values[i];
    }
    return sum / this->values.size();
  }

  /// Defines the number of values
  void setSize(size_t size) {
    this->size = size;
  }

 protected:
  std::list<N> values;
  size_t size = 0;;
};

}  // namespace tinyrobotics