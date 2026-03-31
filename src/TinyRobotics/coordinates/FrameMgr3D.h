#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "TinyRobotics/coordinates/Coordinates.h"
#include "TinyRobotics/utils/Common.h"
#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {

/**
 * @brief Represents a 3D rigid body transform: translation (tx, ty, tz) and
 * quaternion rotation (qx, qy, qz, qw).
 */
struct Transform3D {
  float tx = 0, ty = 0, tz = 0;
  float qx = 0, qy = 0, qz = 0, qw = 1;

  static Transform3D identity() { return {}; }

  // Quaternion multiplication and translation composition
  Transform3D operator*(const Transform3D& other) const {
    Transform3D r;
    // Quaternion multiplication
    r.qw = qw * other.qw - qx * other.qx - qy * other.qy - qz * other.qz;
    r.qx = qw * other.qx + qx * other.qw + qy * other.qz - qz * other.qy;
    r.qy = qw * other.qy - qx * other.qz + qy * other.qw + qz * other.qx;
    r.qz = qw * other.qz + qx * other.qy - qy * other.qx + qz * other.qw;

    // Rotate other's translation by this quaternion and add to this translation
    float x = other.tx, y = other.ty, z = other.tz;
    float ix = qw * x + qy * z - qz * y;
    float iy = qw * y + qz * x - qx * z;
    float iz = qw * z + qx * y - qy * x;
    float iw = -qx * x - qy * y - qz * z;
    r.tx = tx + (ix * qw + iw * -qx + iy * -qz - iz * -qy);
    r.ty = ty + (iy * qw + iw * -qy + iz * -qx - ix * -qz);
    r.tz = tz + (iz * qw + iw * -qz + ix * -qy - iy * -qx);

    return r;
  }

  // Inverse of the transform
  Transform3D inverse() const {
    Transform3D r;
    // Inverse rotation (conjugate)
    r.qx = -qx;
    r.qy = -qy;
    r.qz = -qz;
    r.qw = qw;
    // Inverse translation
    float x = -tx, y = -ty, z = -tz;
    float ix = r.qw * x + r.qy * z - r.qz * y;
    float iy = r.qw * y + r.qz * x - r.qx * z;
    float iz = r.qw * z + r.qx * y - r.qy * x;
    float iw = -r.qx * x - r.qy * y - r.qz * z;
    r.tx = (ix * r.qw + iw * -r.qx + iy * -r.qz - iz * -r.qy);
    r.ty = (iy * r.qw + iw * -r.qy + iz * -r.qx - ix * -r.qz);
    r.tz = (iz * r.qw + iw * -r.qz + ix * -qy - iy * -qx);
    return r;
  }

  /**
   * @brief Applies this transform to a 3D point (x, y, z).
   * Rotates the point by the quaternion and then translates it.
   * @param point Array of 3 floats: {x, y, z}
   * @return Transformed point as std::array<float, 3>
   */
  std::array<float, 3> apply(const std::array<float, 3>& point) const {
    // Quaternion rotation
    float x = point[0], y = point[1], z = point[2];
    // Quaternion-vector multiplication (q * v * q^-1)
    float qx2 = qx + qx, qy2 = qy + qy, qz2 = qz + qz;
    float xx = qx * qx2, yy = qy * qy2, zz = qz * qz2;
    float xy = qx * qy2, xz = qx * qz2, yz = qy * qz2;
    float wx = qw * qx2, wy = qw * qy2, wz = qw * qz2;
    float rx = (1 - (yy + zz)) * x + (xy - wz) * y + (xz + wy) * z;
    float ry = (xy + wz) * x + (1 - (xx + zz)) * y + (yz - wx) * z;
    float rz = (xz - wy) * x + (yz + wx) * y + (1 - (xx + yy)) * z;
    return {rx + tx, ry + ty, rz + tz};
  }
};

/**
 * @class Frame3D
 * @ingroup coordinates
 * @brief Represents a 3D coordinate frame in a hierarchical frame tree.
 *
 * Frame3D models a reference frame in 3D space, such as a robot base, sensor,
 * or world frame. Each frame can have a parent frame, forming a tree structure
 * for managing relative poses. The pose of the frame relative to its parent is
 * described by a Transform3D (translation and quaternion rotation).
 */
class Frame3D {
 public:
  Frame3D(FrameType type, int index = 0, Frame3D* parent = nullptr,
          const Transform3D& tf = Transform3D::identity())
      : type(type), index(index), parent(parent), tf(tf) {}

  void setParent(Frame3D* p) { parent = p; }
  void setTransform(const Transform3D& t) { tf = t; }
  Frame3D* getParent() const { return parent; }
  const Transform3D& getTransform() const { return tf; }
  FrameType getType() const { return type; }
  int getIndex() const { return index; }

 protected:
  FrameType type;
  int index;
  Frame3D* parent;
  Transform3D tf;
};

/**
 * @class FrameMgr3D
 * @ingroup coordinates
 * @brief Manages a hierarchy of 3D coordinate frames and enables SE(3)
 * transforms.
 *
 * FrameMgr3D is designed for robotics and navigation applications where
 * multiple coordinate frames (such as world, robot base, and sensors) must be
 * related and transformed. It supports:
 *   - Defining a tree of 3D frames, each with a pose (translation and
 * quaternion rotation) relative to its parent.
 *   - Computing the SE(3) transform between any two frames in the hierarchy.
 *
 * Example:
 * @code
 * FrameMgr3D mgr;
 * Frame3D world(FrameType3D::WORLD);
 * Frame3D base(FrameType3D::BASE_LINK, 0, &world, Transform3D{1,2,0,0,0,0,1});
 * Frame3D camera(FrameType3D::CAMERA, 0, &base,
 * Transform3D{0.1,0,0.2,0,0,0,1}); Transform3D tf = mgr.getTransform(camera,
 * world);
 * @endcode
 * @ingroup coordinates
 */
class FrameMgr3D {
 public:
  // Returns the transform from 'from' to 'to'
  Transform3D getTransform(const Frame3D& from, const Frame3D& to) const {
    std::vector<const Frame3D*> fromPath = getPathToRoot(&from);
    std::vector<const Frame3D*> toPath = getPathToRoot(&to);

    // Find common ancestor
    int i = fromPath.size() - 1;
    int j = toPath.size() - 1;
    while (i >= 0 && j >= 0 && fromPath[i] == toPath[j]) {
      i--;
      j--;
    }
    i++;
    j++;

    // Compose transforms up from 'from' to ancestor
    Transform3D tf_from = Transform3D::identity();
    for (int k = 0; k < i; ++k) {
      tf_from = fromPath[k]->getTransform() * tf_from;
    }
    // Compose transforms up from 'to' to ancestor
    Transform3D tf_to = Transform3D::identity();
    for (int k = 0; k < j; ++k) {
      tf_to = toPath[k]->getTransform() * tf_to;
    }
    // The transform from 'from' to 'to' is tf_to.inverse() * tf_from
    return tf_to.inverse() * tf_from;
  }

  // GPS/geodetic integration
 protected:
  Frame3D* p_gpsParent = nullptr;
  GPSCoordinate gpsCoordinate;
  float gpsRotationDeg = 0.0f;  // Rotation from local x to GPS north (degrees)

 public:
  /**
   * @brief Sets the GPS reference for a frame.
   * @param gpsParent The frame to associate with the GPS reference.
   * @param gps The GPS coordinate at the origin of gpsParent.
   * @param rotationDeg Rotation from local x to GPS north (degrees).
   */
  void setGPS(Frame3D& gpsParent, const GPSCoordinate& gps,
              float rotationDeg = 0.0f) {
    p_gpsParent = &gpsParent;
    gpsCoordinate = gps;
    gpsRotationDeg = rotationDeg;
  }

  /**
   * @brief Converts a coordinate in the given frame to a GPS coordinate.
   * @param frame The frame in which the coordinate is expressed.
   * @param local The coordinate in the local frame (meters).
   * @return The corresponding GPS coordinate.
   */
  GPSCoordinate toGPS(const Frame3D& frame,
                      const std::array<float, 3>& local) const {
    if (!p_gpsParent) return gpsCoordinate;
    // Get transform from local frame to GPS parent frame
    Transform3D tf = getTransform(frame, *p_gpsParent);
    // Apply the transform to the local coordinate to get position in GPS parent
    // frame
    std::array<float, 3> xyz = tf.apply(local);
    // Rotate by gpsRotationDeg to align with GPS north (in x/y plane)
    float theta_rad = gpsRotationDeg * M_PI / 180.0f;
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);
    float dx = cos_theta * xyz[0] - sin_theta * xyz[1];
    float dy = sin_theta * xyz[0] + cos_theta * xyz[1];
    float dz = xyz[2];
    // Use GPSCoordinate::navigate for conversion (distance in meters, bearing
    // in degrees)
    float distance = sqrt(dx * dx + dy * dy);
    float bearing_deg = atan2(dx, dy) * 180.0 / M_PI;
    return gpsCoordinate.navigate(distance, bearing_deg, dz);
  }

 protected:
  // Returns the path from the given frame up to the root (including self)
  std::vector<const Frame3D*> getPathToRoot(const Frame3D* frame) const {
    std::vector<const Frame3D*> path;
    while (frame) {
      path.push_back(frame);
      frame = frame->getParent();
    }
    return path;
  }
};

}  // namespace tinyrobotics