#pragma once

namespace tinyrobotics {

enum class TimeUnit { S, MS, US, MIN, HOUR };

/**
 * @class Time
 * @ingroup units
 * @brief Represents a time duration with a specific unit (seconds, milliseconds, microseconds, minutes, or hours).
 *
 * Provides methods to set and get the time in different units, as well as handle unit conversion when
 * retrieving the time in a desired unit. The internal representation of the time is stored in the unit
 * specified at construction, and all operations ensure that the time can be accurately converted between
 * supported units.
 *
 * This class can be used for various applications such as navigation, robotics, or any scenario that requires
 * time representation and manipulation. The getValue method allows for easy retrieval of the time in the desired
 * unit, while the setValue method allows for updating the time measurement with a new value and unit. The class
 * is designed to be simple and efficient for use in embedded systems, with basic unit conversion logic that covers
 * common time units.
 */
class Time {
 public:
  Time() = default;
  Time(float time, TimeUnit unit) { setValue(time, unit); }

  void setValue(float newTime, TimeUnit newUnit) {
    time = newTime;
    unit = newUnit;
  }

  float getValue(TimeUnit desiredUnit) const {
    if (unit == desiredUnit) return time;
    switch (unit) {
      case TimeUnit::S:
        if (desiredUnit == TimeUnit::MS) return time * 1000;
        if (desiredUnit == TimeUnit::US) return time * 1000000;
        if (desiredUnit == TimeUnit::MIN) return time / 60;
        if (desiredUnit == TimeUnit::HOUR) return time / 3600;
        break;
      case TimeUnit::MS:
        if (desiredUnit == TimeUnit::S) return time / 1000;
        if (desiredUnit == TimeUnit::US) return time * 1000;
        if (desiredUnit == TimeUnit::MIN) return time / 60000;
        if (desiredUnit == TimeUnit::HOUR) return time / 3600000;
        break;
      case TimeUnit::US:
        if (desiredUnit == TimeUnit::S) return time / 1000000;
        if (desiredUnit == TimeUnit::MS) return time / 1000;
        if (desiredUnit == TimeUnit::MIN) return time / 60000000;
        if (desiredUnit == TimeUnit::HOUR) return time / 3600000000;
        break;
      case TimeUnit::MIN:
        if (desiredUnit == TimeUnit::S) return time * 60;
        if (desiredUnit == TimeUnit::MS) return time * 60000;
        if (desiredUnit == TimeUnit::US) return time * 60000000;
        if (desiredUnit == TimeUnit::HOUR) return time / 60;
        break;
      case TimeUnit::HOUR:
        if (desiredUnit == TimeUnit::S) return time * 3600;
        if (desiredUnit == TimeUnit::MS) return time * 3600000;
        if (desiredUnit == TimeUnit::US) return time * 3600000000;
        if (desiredUnit == TimeUnit::MIN) return time * 60;
        break;
    }
    return -1;  // Invalid conversion
  }

 protected:
  float time = 0.0f;
  TimeUnit unit = TimeUnit::MS;
};

}  // namespace tinyrobotics