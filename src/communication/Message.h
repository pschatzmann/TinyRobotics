#pragma once
#include "utils/Common.h"

namespace tinyrobotics {

enum class MessgeOrigin {
  RemoteControl,  ///< Message from a remote control interface (e.g., RC receiver)
  Autonomy,       ///< Message from an autonomous control module (e.g., path planner)
  Sensor,         ///< Message from a sensor module (e.g., IMU, GPS)
  System,         ///< Internal system message (e.g., status update)
  Motor,          ///< Message from a motor controller (e.g., ESC)
  Servo,          ///< Message from a servo controller
  Rudder,         ///< Message from a rudder controller (specific to boats/planes)
  Aileron,        ///< Message from an aileron controller (specific to planes)
  Elevator,       ///< Message from an elevator controller (specific to planes)
  IMU,            ///< Message from an IMU sensor
};

/**
 * @brief Types of message content for communication between modules or devices.
 */
enum class MessageContent {
  Pitch,          ///< Pitch angle or command
  Roll,           ///< Roll angle or command
  Yaw,            ///< Yaw angle or command
  Throttle,       ///< Throttle or power command
  Speed,          ///< Linear speed
  SteeringAngle,  ///< Steering angle (e.g., for Ackerman vehicles)
  Turn,           ///< Turn command (e.g., for differential drive)
  Heading,        ///< Heading angle (e.g., compass direction)
  MotorSpeed,     ///< Motor speed (RPM or percent)
  Position,       ///< Position data (coordinates)
  PositionGPS,    ///< Position data from GPS (latitude, longitude)
};

/**
 * @brief Generic message structure for communication, parameterized by value type.
 *
 * @tparam T Type of the value (default: float)
 *
 * @note When using Message in function signatures (e.g., virtual functions), always use Message<float> explicitly.
 *       C++ requires the template argument in these contexts, even though the default is float.
 */
template <typename T = float>
struct Message {
  const char* prefix = "MSG";  ///< prefix for message identification
  uint8_t size = sizeof(Message);
  MessgeOrigin source = MessgeOrigin::RemoteControl;  ///< Source of the message
  uint8_t source_id = 0;   /// Optional identifier for the source (e.g., sensor ID, motor ID)
  MessageContent content;  ///< Type of message content
  Unit unit;               ///< Unit of the value
  T value;                 ///< Value of the message

  /**
   * @brief Default constructor.
   */
  Message() = default;

  /**
   * @brief Construct a message with content, value, and unit.
   * @param c Message content type
   * @param v Value
   * @param u Unit
   */
  Message(MessageContent c, float v, Unit u) : content(c), value(v), unit(u) {}
};

}  // namespace tinyrobotics