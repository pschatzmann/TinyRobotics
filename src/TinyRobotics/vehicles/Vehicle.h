#pragma once

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include <vector>

namespace tinyrobotics {

/**
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

};

}  // namespace tinyrobotics