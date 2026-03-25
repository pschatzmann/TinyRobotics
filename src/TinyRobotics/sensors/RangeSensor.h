#pragma once
#include <cmath>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/FrameMgr2D.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {

/**
 * @brief A simple range sensor class that can be used to represent a lidar or
 * ultrasonic sensor. It holds the distance measured by the sensor, the angle of
 * obstacle in degrees, and the transform from the sensor frame to the world
 * frame. The class provides methods to set the distance, field of view, and
 * transform, as well as a method to get the obstacle coordinate in the world
 * frame based on the current distance and transform. The sensor reading is
 * considered valid if the distance is greater than 0. This class can be used in
 * a robotics application to represent the data from a range sensor and to
 * calculate the position of detected obstacles in the world frame for
 * navigation and obstacle avoidance purposes.
 */
template <typename T = DistanceM>
class RangeSensor {
 public:
  RangeSensor(const Transform2D& tf, float obstacleDegree = 0) {
    setObstacleDegree(obstacleDegree);
    setTransform(tf);
  }
  RangeSensor(float obstacleDegree = 0) { setObstacleDegree(obstacleDegree); }

  /// Defines the angle to the obstacle in degrees: 0 means forward
  void setObstacleDegree(float deg) { obstacle_deg_ = deg; }

  /// Get the angle of the obstacle relative to the sensor's forward direction
  /// in degrees.
  float getObstacleDegree() const { return obstacle_deg_; }

  /// Set the distance measured by the sensor.
  void setDistance(float distance) { this->distance = distance; }

  /// Provide the distance measured by the sensor. In a real implementation,
  /// this would
  float getDistance() const { return distance; }

  /// Define the lidar to world transform
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
    float theta = obstacle_deg_ * M_PI / 180.0f;
    Coordinate<T> obstacle_lidar(distance * cos(theta), distance * sin(theta));
    result = lidar_to_world_tf.apply(obstacle_lidar);
    return true;
  }

  /// Check if the sensor reading is valid (distance > 0)
  operator bool() const { return distance > 0; }  // Valid if distance is set

  /// Return true if there is an obstacle detected (distance > 0)
  bool hasObstacle() const { return distance > 0; }

 protected:
  float obstacle_deg_ = 0;
  float distance = 0;
  Transform2D lidar_to_world_tf;
  bool has_transform_ = false;
};

}  // namespace tinyrobotics