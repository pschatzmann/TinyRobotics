#pragma once
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

/**
 * @enum MessageOrigin
 * @ingroup communications
 * @brief Source/origin of a message in the communication framework.
 */
enum class MessageOrigin {
  Undefined = 0,
  RemoteControl,  ///< Message from a remote control interface (e.g., RC
                  ///< receiver)
  Autonomy,  ///< Message from an autonomous control module (e.g., path planner)
  Sensor,    ///< Message from a sensor module (e.g., IMU, GPS)
  System,    ///< Internal system message (e.g., status update)
  Motor,     ///< Message from a motor controller (e.g., ESC)
  Servo,     ///< Message from a servo controller
  Rudder,    ///< Message from a rudder controller (specific to boats/planes)
  Aileron,   ///< Message from an aileron controller (specific to planes)
  Elevator,  ///< Message from an elevator controller (specific to planes)
  IMU,       ///< Message from an IMU sensor
  LIDAR,     ///< Message from a LIDAR or IR distance sensor
  Camera,    ///< Message from a camera or vision sensor
  GPS,       ///< Message from a GPS module
};

/**
 * @enum MessageContent
 * @ingroup communications
 * @brief Types of message content for communication between modules or devices.
 */
enum class MessageContent {
  Undefined = 0,
  Angle,          ///< Generic angle (e.g., for orientation or steering)
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
  Distance,       ///< Distance measurement (e.g., from a range sensor)
  Temperature,    ///< Temperature reading (e.g., from a temperature sensor)
  Error,          ///< Error value (e.g., for PID control)
  Density         ///< Density value (e.g., for obstacle detection)
};

/**
 * @class Message
 * @ingroup communication
 * @brief Generic message structure for communication, parameterized by value
 * type.
 * @tparam T Type of the value (default: float)
 *
 * @note When using Message in function signatures (e.g., virtual functions),
 * always use Message<float> explicitly. C++ requires the template argument in
 * these contexts, even though the default is float.
 */
template <typename T = float>
struct Message {
  const char* prefix = "MSG";  ///< prefix for message identification
  uint8_t size = sizeof(Message);
  MessageOrigin origin =
      MessageOrigin::RemoteControl;  ///< Source of the message. @see
                                     ///< MessageOrigin
  uint8_t origin_id =
      0;  /// Optional identifier for the source (e.g., sensor ID, motor ID)
  MessageContent content;  ///< Type of message content. @see MessageContent
  Unit unit;               ///< Unit of the value
  T value{};               ///< Value of the message

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
  Message(MessageContent c, T v, Unit u) : content(c), unit(u), value(v) {}
  Message(MessageContent c, T v, Unit u, MessageOrigin orig)
    : origin(orig), content(c), unit(u), value(v) {}
};

}  // namespace tinyrobotics