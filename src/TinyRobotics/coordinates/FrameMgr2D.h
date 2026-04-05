#pragma once
#include <cmath>
#include <vector>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/GPSCoordinate.h"
#include "TinyRobotics/utils/Common.h"
#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {

/**
 * @class Transform2D
 * @ingroup coordinates
 * @brief Represents a 2D rigid body transform (SE(2)): translation and
 * rotation.
 *
 * Transform2D models a pose in 2D space, consisting of a translation (x, y) in
 * meters and a heading (rotation) in degrees. It supports:
 *   - Applying the transform to local coordinates (rotating and translating
 * them into the global frame)
 *   - Composing transforms (chaining multiple poses)
 *   - Inverting the transform (computing the reverse transformation)
 *
 * The heading is in degrees, where 0 means facing along the positive x-axis,
 * and positive values rotate counterclockwise (e.g., 90 degrees is along the
 * positive y-axis).
 *
 * Typical usage:
 *   - Represent the pose of a robot, sensor, or frame in 2D space.
 *   - Transform points between local and global frames.
 *   - Chain multiple transforms for hierarchical coordinate systems.
 *
 * Example:
 * @code
 * Transform2D base_to_world(1.0, 2.0, 45); // 1m x, 2m y, 45 deg rotation
 * Coordinate local_point(0.5, 0.0); // 0.5m ahead in base frame
 * Coordinate global_point = base_to_world.apply(local_point);
 * Transform2D world_to_base = base_to_world.inverse();
 * @endcode
 */
class Transform2D {
 public:
  Transform2D() : pos(0, 0), heading_deg(0) {}
  Transform2D(float x, float y, float heading_deg)
      : pos(x, y), heading_deg(heading_deg) {}
  Transform2D(const Coordinate<DistanceM>& coord, float heading_deg)
      : pos(coord), heading_deg(heading_deg) {}

  Coordinate<DistanceM> pos;  ///< Position (x, y) in meters
  float heading_deg;          ///< Heading in degrees (0 = x axis)

  /// Get heading in specified unit (degrees or radians)
  float getHeading(AngleUnit unit) const {
    return Angle(heading_deg, AngleUnit::DEG).getValue(unit);
  }

  /**
   * @brief  Applies this transform to a local coordinate, returning the
   * coordinate in the global frame. The local coordinate is first rotated by
   * the heading, then translated by the position of this transform.
   */
  Coordinate<DistanceM> apply(const Coordinate<DistanceM>& local) const {
    float theta = heading_deg * static_cast<float>(M_PI) / 180.0f;
    float x = std::cos(theta) * local.x - std::sin(theta) * local.y + pos.x;
    float y = std::sin(theta) * local.x + std::cos(theta) * local.y + pos.y;
    return Coordinate<DistanceM>(x, y);
  }

  /**
   * @brief Compose this transform with another (this * other).
   * Applies this transform, then the other.
   */
  Transform2D operator*(const Transform2D& other) const {
    float theta_rad = heading_deg * static_cast<float>(M_PI) / 180.0f;
    float cos_theta = std::cos(theta_rad);
    float sin_theta = std::sin(theta_rad);
    float nx = pos.x + cos_theta * other.pos.x - sin_theta * other.pos.y;
    float ny = pos.y + sin_theta * other.pos.x + cos_theta * other.pos.y;
    float nheading = heading_deg + other.heading_deg;
    nheading = normalizeAngleDeg(nheading);
    return Transform2D(nx, ny, nheading);
  }

  /**
   * @brief Invert this transform.
   */
  Transform2D inverse() const {
    float theta_rad = heading_deg * static_cast<float>(M_PI) / 180.0f;
    float cos_theta = std::cos(theta_rad);
    float sin_theta = std::sin(theta_rad);
    float ix = -pos.x * cos_theta - pos.y * sin_theta;
    float iy = pos.x * sin_theta - pos.y * cos_theta;
    float iheading = -heading_deg;
    iheading = normalizeAngleDeg(iheading);
    return Transform2D(ix, iy, iheading);
  }
};

/**
 * @brief Represents a 2D coordinate frame in a hierarchical frame tree.
 *
 * Frame2D models a reference frame in 2D space, such as a robot base, sensor,
 * or world frame. Each frame can have a parent frame, forming a tree structure
 * for managing relative poses. The pose of the frame relative to its parent is
 * described by a Transform2D (translation and heading).
 *
 * Features:
 *   - Stores frame type and index for identification (e.g., WORLD, BASE,
 * SENSOR).
 *   - Maintains a pointer to its parent frame (if any).
 *   - Stores the SE(2) transform (Transform2D) from this frame to its parent.
 *   - Supports setting and retrieving the transform and parent.
 *   - Used by FrameMgr2D to compute transforms and manage coordinate
 * conversions.
 *
 * Typical usage:
 *   - Define a hierarchy of frames for a robot (e.g., world → base → sensor).
 *   - Set the transform from each frame to its parent.
 *   - Use with FrameMgr2D to compute transforms between arbitrary frames and
 * convert to GPS.
 *
 * Example:
 * @code
 * Frame2D world(FrameType::WORLD);
 * Frame2D base(FrameType::BASE, 0, world, Transform2D(1.0, 2.0, 45));
 * Frame2D lidar(FrameType::SENSOR, 0, base, Transform2D(0.2, 0.0, 0));
 * @endcode
 */
struct Frame2D {
  Frame2D(FrameType type, uint8_t idx = 0) : type(type), index(idx) {}
  Frame2D(FrameType type, uint8_t idx, Frame2D& parent)
      : type(type), index(idx), p_parent(&parent) {}
  Frame2D(FrameType type, uint8_t idx, Frame2D& parent, const Transform2D& tf)
      : type(type), index(idx), p_parent(&parent), tf(tf) {}

  bool operator==(const Frame2D& o) const {
    return type == o.type && index == o.index;
  }

  FrameType getType() const { return type; }
  uint8_t getIndex() const { return index; }
  const Frame2D* getParent() const { return p_parent; }
  void setParent(Frame2D& parent) { p_parent = &parent; }
  const Transform2D& getTransform() const { return tf; }
  Transform2D& getTransform() { return tf; }
  void setTransform(const Transform2D& transform) { tf = transform; }

 protected:
  FrameType type;
  uint8_t index = 0;
  Frame2D* p_parent = nullptr;
  Transform2D tf;
};

/**
 * @class FrameMgr2D
 * @ingroup coordinates
 * @brief Manages a hierarchy of 2D coordinate frames and enables SE(2)
 * transforms and GPS conversion.
 *
 * FrameMgr2D is designed for robotics and navigation applications where
 * multiple coordinate frames (such as world, robot base, and sensors) must be
 * related and transformed. It supports:
 *   - Defining a tree of 2D frames, each with a pose (translation and heading)
 * relative to its parent.
 *   - Computing the SE(2) transform (translation + rotation) between any two
 * frames in the hierarchy.
 *   - Associating a GPS coordinate and orientation with any frame, enabling
 * conversion of local coordinates to global GPS coordinates.
 *   - Converting local frame coordinates to GPS using the reference frame and
 * orientation.
 *
 * Typical usage:
 *   - Model a robot's world, base, and sensor frames.
 *   - Set the GPS reference for the world or another frame.
 *   - Compute the transform between frames for sensor fusion, mapping, or
 * control.
 *   - Convert robot or sensor positions to GPS for navigation or logging.
 *
 * Example:
 * @code
 * FrameMgr2D mgr;
 * Frame2D world(FrameType::WORLD);
 * Frame2D base(FrameType::BASE, 0, world, Transform2D(1.0, 2.0, 45));
 * mgr.setGPS(world, GPSCoordinate(48.8584, 2.2945, 35)); // Eiffel Tower
 * Coordinate robot_local(0.5, 0.0); // 0.5m ahead in base frame
 * // Convert robot position in base frame to GPS
 * GPSCoordinate robot_gps = mgr.toGPS(base);
 * @endcode
 *
 * This class is suitable for embedded and desktop robotics systems that require
 * flexible, hierarchical frame management and integration with GPS.
 * @ingroup coordinates
 */
class FrameMgr2D {
 public:
  FrameMgr2D() = default;

  /// Defines the GPI coordinate for the indicated parent frame. The GPS
  /// coordinate is assumed to be facing north (90 degrees) by default, but a
  /// different rotation can be specified if needed.
  void setGPS(Frame2D& parent, const GPSCoordinate gps,
              float rotationDeg = 90) {
    p_gpsParent = &parent;
    gpsCoordinate = gps;
    gpsRotationDeg = rotationDeg;
  }

  /// Converts a local frame to a GPS coordinate
  GPSCoordinate toGPS(const Frame2D& frame) const {
    if (!p_gpsParent) {
      TRLogger.error(
          "FrameMgr2D: GPS parent frame not set, cannot convert to GPS");
      return GPSCoordinate();  // Return default GPS coordinate
    }
    Transform2D tf = getTransform(frame, *p_gpsParent);

    // Rotate the local (x, y) by gpsRotationDeg to align with GPS north
    float theta_rad = gpsRotationDeg * static_cast<float>(M_PI) / 180.0f;
    float cos_theta = std::cos(theta_rad);
    float sin_theta = std::sin(theta_rad);
    float dx = cos_theta * tf.pos.x - sin_theta * tf.pos.y;
    float dy = sin_theta * tf.pos.x + cos_theta * tf.pos.y;

    float distance = std::sqrt(dx * dx + dy * dy);
    float bearing_deg = std::atan2(dx, dy) * 180.0f /
                        static_cast<float>(M_PI);  // Bearing from north
    bearing_deg = normalizeAngleDeg(bearing_deg);

    return gpsCoordinate.navigate(distance, bearing_deg);
  }

  /**
   * @brief Computes the transform from one frame to another using SE(2) math.
   * @param from The starting frame.
   * @param to The target frame.
   * @return The composed transform from 'from' to 'to'.
   */
  Transform2D getTransform(const Frame2D& from, const Frame2D& to) const {
    // Find paths from both frames to root
    std::vector<const Frame2D*> from_path = getPathToRoot(from);
    std::vector<const Frame2D*> to_path = getPathToRoot(to);

    // Find lowest common ancestor (LCA)
    int i = from_path.size() - 1;
    int j = to_path.size() - 1;
    const Frame2D* lca = nullptr;
    while (i >= 0 && j >= 0 && from_path[i] == to_path[j]) {
      lca = from_path[i];
      --i;
      --j;
    }

    // Compose transform from 'from' up to LCA (invert each step)
    Transform2D tf_from;
    for (int k = 0; k <= i; ++k) {
      tf_from = from_path[k]->getTransform() * tf_from;
    }
    tf_from = tf_from.inverse();

    // Compose transform from LCA down to 'to'
    Transform2D tf_to;
    for (int k = j; k >= 0; --k) {
      tf_to = tf_to * to_path[k]->getTransform();
    }

    // Final transform: from → LCA (inverted) → to
    return tf_to * tf_from;
  }

 protected:
  Frame2D* p_gpsParent = nullptr;
  GPSCoordinate gpsCoordinate;
  float gpsRotationDeg;

  /**
   * @brief Returns the path from the given frame up to the root (including
   * self). The path is verly small, so no special optimizations are needed.
   */
  std::vector<const Frame2D*> getPathToRoot(const Frame2D& frame) const {
    std::vector<const Frame2D*> path;
    const Frame2D* current = &frame;
    while (current) {
      path.push_back(current);
      current = current->getParent();
    }
    return path;
  }
};

}  // namespace tinyrobotics
