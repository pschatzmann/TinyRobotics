#pragma once

#include <vector>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"

namespace tinyrobotics {

/**
 * @class Vehicle
 * @ingroup vehicles
 * @brief Abstract base class for all vehicle types.
 *
 * The Vehicle class defines a common interface for all robotic vehicles in the
 * library. It enforces the implementation of a reset() method, which should
 * reset the vehicle's actuators and internal state to a safe or neutral
 * configuration.
 *
 * All vehicle classes (e.g., Car4WD, Quadrotor, AirPlane, MotorBoat) should
 * inherit from Vehicle.
 */
class Vehicle : public MessageHandler, public MessageSource {
 public:
  /**
   * @brief Reset the vehicle to a safe or neutral state (pure virtual).
   */
  virtual void end() = 0;

  /**
   * @brief Check if the necessary pins for the vehicle's actuators have been
   * set (pure virtual).
   */
  virtual bool isPinsSet() const = 0;

  virtual std::vector<MessageContent> getControls() const = 0;

  bool isValidMessageSource(MessageOrigin origin) const {
    return origin == MessageOrigin::RemoteControl ||
           origin == MessageOrigin::Autonomy;
  }

  /**
   * @brief Get the speed factor (scaling for speed commands).
   */
  float getSpeedFactor() const { return speedFactor_; }

  /**
   * @brief Set the speed factor (scaling for speed commands).
   */
  void setSpeedFactor(float factor) { speedFactor_ = factor; }
 protected:
  float speedFactor_ = 1.0f;

};

}  // namespace tinyrobotics