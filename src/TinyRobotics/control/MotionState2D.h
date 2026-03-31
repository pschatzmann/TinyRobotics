#pragma once
#include "TinyRobotics/units/Units.h"
#include "TinyRobotics/coordinates/Coordinate.h"

namespace tinyrobotics {

/**
 * @struct Delta2D
 * @ingroup control
 * @brief Represents a 2D incremental motion update (dx, dy, dtheta).
 *
 * Used for odometry, IMU, and motion estimation to describe the change in position and orientation
 * over a time step in 2D space.
 */
struct Delta2D {
  float dx;
  float dy;
  float dtheta;
};

/**
 * @class MotionState2D
 * @ingroup control
 * @brief Interface for representing the navigation state of a robot in 2D space.
 *
 * Provides access to position, heading (orientation), and speed.
 */
/**
 * @class IMotionState2D
 * @ingroup control
 * @brief Interface for representing the navigation state of a robot in 2D space.
 *
 * Provides access to position, heading (orientation), and speed.
 */
class IMotionState2D {
  public:
    virtual Coordinate<DistanceM> getPosition() const = 0;
    virtual Angle getHeading() const = 0;
    virtual Speed getSpeed() const = 0;
};

/**
 * @brief Concrete implementation of INavigationState2D for storing and accessing 2D navigation state.
 *
 * Holds position, heading, and speed values for a robot in 2D space.
 */
class MotionState2D : public IMotionState2D {
  public:
    /**
     * @brief Construct a new NavigationState2D object.
     * @param position The 2D position (meters).
     * @param heading The heading/orientation (radians).
     * @param speed The speed (meters/second).
     */
    MotionState2D(const Coordinate<DistanceM>& position, const Angle& heading, const Speed& speed)
        : position(position), heading(heading), speed(speed) {}

    Coordinate<DistanceM> getPosition() const override { return position; }
    Angle getHeading() const override { return heading; }
    Speed getSpeed() const override { return speed; }

  protected:
    Coordinate<DistanceM> position;
    Angle heading;
    Speed speed;
};

}  // namespace tinyrobotics