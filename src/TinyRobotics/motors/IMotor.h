#pragma once

#include "TinyRobotics/utils/Config.h"

namespace tinyrobotics {

/**
 * @class IMotor
 * @ingroup motors
 * @brief Abstract base class for all motor driver types.
 *
 * The Motor class defines a common interface for all motor drivers in the
 * library. It enforces the implementation of:
 *   - end(): to safely stop or release the motor hardware
 *   - isPinsSet(): to check if the required pins have been configured
 *
 * All specific motor driver classes (e.g., HBridge, ServoMotor) should inherit
 * from Motor.
 */
template <typename T = float>
class IMotor {
 public:
  IMotor() = default;
  virtual bool begin() = 0;
  virtual void end() = 0;
  virtual bool isPinsSet() const = 0;
  virtual bool setValuePercent(T percent) = 0;
  virtual T getValuePercent() const = 0;
  void setID(uint8_t id) { this->id = id; }
  uint8_t getID() const { return id; }

 protected:
  uint8_t id = 0;  // Optional ID for identifying the motor
};

}  // namespace tinyrobotics