#pragma once
#include <cmath>

namespace tinyrobotics {

enum class AngularVelocityUnit { RadPerSecond, DegPerSecond };

class AngularVelocity {
 public:
  AngularVelocity() = default;
  AngularVelocity(float angularVelocity, AngularVelocityUnit unit)
      : angularVelocity(angularVelocity), unit(unit) {}

  float getValue(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return angularVelocity;
    switch (unit) {
      case AngularVelocityUnit::RadPerSecond:
        if (desiredUnit == AngularVelocityUnit::DegPerSecond)
          return angularVelocity * 180.0f / M_PI;
        break;
      case AngularVelocityUnit::DegPerSecond:
        if (desiredUnit == AngularVelocityUnit::RadPerSecond)
          return angularVelocity * M_PI / 180.0f;
        break;
    }
    return 0;  // Invalid conversion
  }

 protected:
  float angularVelocity = 0.0f;
  AngularVelocityUnit unit = AngularVelocityUnit::RadPerSecond;
};

class AngularVelocity3D {
 public:
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  AngularVelocityUnit unit = AngularVelocityUnit::RadPerSecond;

  AngularVelocity3D() = default;
  AngularVelocity3D(float x, float y, float z, AngularVelocityUnit unit)
      : x(x), y(y), z(z), unit(unit) {}

  float getX(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return x;
    AngularVelocity temp(x, unit);
    return temp.getValue(desiredUnit);
  }
  float getY(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return y;
    AngularVelocity temp(y, unit);
    return temp.getValue(desiredUnit);
  }
  float getZ(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return z;
    AngularVelocity temp(z, unit);
    return temp.getValue(desiredUnit);
  }

};

}  // namespace tinyrobotics
