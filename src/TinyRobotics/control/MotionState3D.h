#pragma once
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/units/AngularVelocity.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/units/Speed.h"

namespace tinyrobotics {

/**
 * @struct Delta3D
 * @ingroup control
 * @brief Represents a 3D incremental motion update (dx, dy, dz, dyaw, dpitch, droll).
 *
 * Used for odometry, IMU, and motion estimation to describe the change in position and orientation
 * over a time step in 3D space.
 */
struct Delta3D {
  float dx;
  float dy;
  float dz;
  float dyaw;
  float dpitch;
  float droll;
};

/**
 * @class IMotionState3D
 * @ingroup control
 * @brief Interface for representing the motion state of a robot in 3D space.
 *
 * Provides access to position, orientation, linear velocity, and angular velocity.
 */
class IMotionState3D {
 public:
  virtual Coordinate<DistanceM> getPosition() const = 0;
  virtual Orientation3D getOrientation() const = 0;
  virtual Speed3D getSpeed() const = 0;
  virtual AngularVelocity3D getAngularVelocity() const = 0;
};

/**
 * @class MotionState3D
 * @ingroup control
 * @brief Represents the full 3D motion state of a robot or vehicle.
 *
 * Encapsulates position, orientation, linear velocity, and angular velocity in 3D space.
 * Provides a unified interface for state estimation, control, and sensor fusion modules.
 *
 * Used throughout the TinyRobotics framework for odometry, IMU, fusion, and control logic.
 *
 */
class MotionState3D : public IMotionState3D {
public:
  MotionState3D() = default;
  MotionState3D(const Coordinate<DistanceM>& position,
                const Orientation3D& orientation, const Speed3D& speed,
                const AngularVelocity3D& angularVelocity)
      : position(position),
        orientation(orientation),
        speed(speed),
        angularVelocity(angularVelocity) {}

  Coordinate<DistanceM> getPosition() const override { return position; }
  Orientation3D getOrientation() const override { return orientation; }
  Speed3D getSpeed() const override { return speed; }
  AngularVelocity3D getAngularVelocity() const override {
    return angularVelocity;
  }

 protected:
  Coordinate<DistanceM> position;
  Orientation3D orientation;
  Speed3D speed;
  AngularVelocity3D angularVelocity;
};

}  // namespace tinyrobotics
