#pragma once
#include <cmath>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/FrameMgr2D.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {

/**
 * @class RangeSensor
 * @ingroup sensors
 * @brief Generic range sensor abstraction for LIDAR, ultrasonic, or similar
 * sensors.
 *
 * This template class models a simple range sensor, such as a LIDAR or
 * ultrasonic sensor, for robotics applications. It tracks the measured distance
 * to an obstacle, the angle (bearing) of the obstacle relative to the sensor,
 * and the transform from the sensor's local frame to the world frame. The class
 * provides methods to set and retrieve the measured distance and bearing,
 * update the sensor-to-world transform, and compute the world-frame coordinates
 * of detected obstacles.
 *
 * ## Features
 * - Supports any distance unit via template parameter (default: meters)
 * - Publishes distance, angle, and obstacle position as messages (for
 * integration with messaging framework)
 * - Computes obstacle coordinates in the world frame using a 2D transform
 * - Validity checks for sensor readings (distance > 0)
 * - Can be used for navigation, mapping, and obstacle avoidance
 *
 * ## Usage Example
 * @code
 *   Transform2D tf = ...; // Sensor-to-world transform
 *   RangeSensor<> sensor(tf, 0.0f); // 0 degrees = forward
 *   sensor.begin();
 *   sensor.setObstacleDistance(1.5f); // Set measured distance
 *   if (sensor) {
 *     Coordinate<DistanceM> obs;
 *     if (sensor.getObstacleCoordinate(obs)) {
 *       // Use obs for navigation
 *     }
 *   }
 * @endcode
 *
 * ## Template Parameters
 * @tparam T Distance type (default: DistanceM)
 *
 * ## Methods
 * - setObstacleDirectionDegree(deg): Set obstacle bearing in degrees
 * - setObstacleDistance(distance): Set measured distance
 * - setObstacle(degree, distance): Set both bearing and distance
 * - setTransform(tf): Set sensor-to-world transform
 * - getObstacleCoordinate(result): Compute world-frame obstacle coordinate
 * - hasObstacle(): True if a valid obstacle is detected
 *
 * ## Applications
 * - LIDAR, ultrasonic, or IR range sensing
 * - Robot navigation and mapping
 * - Obstacle detection and avoidance
 *
 * @author Phil Schatzmann

 */
template <typename T = DistanceM>
class RangeSensor : public MessageSource {
 public:
  RangeSensor(const Transform2D& tf, float obstacleDegree = 0) {
    setObstacleDirectionDegree(obstacleDegree);
    setTransform(tf);
  }
  RangeSensor(float obstacleDegree = 0) {
    setObstacleDirectionDegree(obstacleDegree);
  }

  /// Start the sensor (e.g., initialize hardware)
  bool begin() {
    is_active_ = true;
    return true;
  }

  /// Update the sensor with angle (deg) and distance (m)
  void update(float angleDeg, float distanceM) {
    setObstacleDirectionDegree(angleDeg);
    setObstacleDistance(distanceM);
  }

  /// Update the sensor with angle and distance objects
  void update(Angle angle, Distance distance) {
    setObstacleDirection(angle);
    setObstacleDistance(distance);
  }

  /// Stop the sensor
  void end() { is_active_ = false; }

  /// Defines the angle to the obstacle in degrees: 0 means forward
  void setObstacleDirectionDegree(float deg) { obstacle_deg_ = deg; }

  /// Set the obstacle direction using an Angle object
  void setObstacleDirection(Angle angle) {
    setObstacleDirectionDegree(angle.getValue(AngleUnit::DEG));
  }

  /// Get the angle of the obstacle relative to the sensor's forward direction
  /// in degrees.
  float getObstacleDirectionDegree() const { return obstacle_deg_; }

  /// Set the distance measured by the sensor. Make sure that the
  /// ObstacleDegree is defined
  bool setObstacleDistance(Distance dist) {
    return setObstacleDistance(dist.getValue(DistanceUnit::M));
  }

  /// Set the distance measured by the sensor. Make sure that the
  /// ObstacleDegree is defined
  bool setObstacleDistance(float distanceM) {
    if (!is_active_) return false;
    this->distanceM = distanceM;

    // If an obstacle is detected (distance > alertDistance), send message
    if (distanceM >= alertDistanceM_ &&
        std::abs(obstacle_deg_) <= alertAngleDeg_) {
      Message<float> msg(MessageContent::Obstacle, distanceM, Unit::Meters);
      msg.origin = MessageOrigin::LIDAR;
      sendMessage(msg);
    }

    // Publish messages for distance, angle, and obstacle coordinate if valid
    Coordinate<T> obstacle;
    if (getObstacleCoordinate(obstacle)) {
      // publish distance
      Message<T> msgDistance(MessageContent::Distance, distanceM, Unit::Meters);
      msgDistance.origin = MessageOrigin::LIDAR;
      sendMessage(msgDistance);

      // publish angle to direction of movement
      Message<T> msgAngle(MessageContent::Angle, obstacle_deg_,
                          Unit::AngleDegree);
      msgAngle.origin = MessageOrigin::LIDAR;
      sendMessage(msgAngle);

      // publish obstacle coordinate
      Message<Coordinate<T>> msgLocation(MessageContent::Position, obstacle,
                                         Unit::Meters);
      msgLocation.origin = MessageOrigin::LIDAR;
      sendMessage(msgLocation);
    }
    return true;
  }

  /// Set the alert angle threshold for obstacle detection
  void setObstacleAlertAngle(Angle deg) {
    alertAngleDeg_ = deg.getValue(AngleUnit::DEG);
  }

  /// Set the alert distance threshold for obstacle detection
  void setObstacleAlertDistance(Distance dist) {
    alertDistanceM_ = dist.getValue(DistanceUnit::M);
  }

  /// Provide the distance measured by the sensor. In a real implementation,
  /// this would
  float getObstacleDistance() const { return distanceM; }

  /// Convenience method to set both the obstacle bearing and distance at once.
  void setObstacle(float degree, float distance) {
    setObstacleDirectionDegree(degree);
    setObstacleDistance(distance);
  }

  /// Set the sensor-to-world transform
  void setTransform(const Transform2D& tf) {
    lidar_to_world_tf = tf;
    has_transform_ = true;
  }

  /// Get the obstacle coordinate in world frame based on the current distance
  /// and transform.
  bool getObstacleCoordinate(Coordinate<T>& result) {
    if (!has_transform_) {
      // Optionally log error
      TRLogger.error(
          "RangeSensor: No transform set, cannot compute obstacle coordinate");
      return false;
    }
    float theta = obstacle_deg_ * static_cast<float>(M_PI) / 180.0f;
    Coordinate<T> obstacle_lidar(distanceM * std::cos(theta),
                                 distanceM * std::sin(theta));
    result = lidar_to_world_tf.apply(obstacle_lidar);
    return true;
  }

  /// Check if the sensor reading is valid (distance > 0)
  operator bool() const { return distanceM > 0; }  // Valid if distance is set

  /// Return true if there is an obstacle detected (distance > 0)
  bool hasObstacle() const { return distanceM > 0; }

  /// Compute a speed factor (0.0 to 1.0) based on the distance to the obstacle
  float getSpeedFactor(Distance breakingDistance) const {
    if (distanceM <= 0) return 1.0f;  // No obstacle, full speed
    if (distanceM >= breakingDistance.getValue(DistanceUnit::M))
      return 1.0f;  // Beyond breaking distance, full speed
    // Linearly scale speed factor based on distance to obstacle
    float factor = distanceM / breakingDistance.getValue(DistanceUnit::M);
    return constrain(factor, 0.0f, 1.0f);
  }

 protected:
  float obstacle_deg_ = 0;
  float distanceM = 0;
  Transform2D lidar_to_world_tf;
  bool has_transform_ = false;
  bool is_active_ = false;
  float alertAngleDeg_ = 5;      // Default to 5 degrees
  float alertDistanceM_ = 1.0f;  // Default to 1 meter

};

}  // namespace tinyrobotics