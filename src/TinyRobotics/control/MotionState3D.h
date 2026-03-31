#pragma once
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/units/AngularVelocity.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/units/Speed.h"

namespace tinyrobotics {

struct Delta3D {
  float dx;
  float dy;
  float dz;
  float dyaw;
  float dpitch;
  float droll;
};

class IMotionState3D {
 public:
  virtual Coordinate<DistanceM> getPosition() const = 0;
  virtual Orientation3D getOrientation() const = 0;
  virtual Speed3D getSpeed() const = 0;
  virtual AngularVelocity3D getAngularVelocity() const = 0;
};

class MotionState3D : public IMotionState3D {
 public:
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
