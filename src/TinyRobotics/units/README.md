# TinyRobotics Units Module

This directory provides strongly-typed units for distance, angle, speed, and time, ensuring type safety and clarity in robotics, navigation, and control code.

## Contents

- **Distance.h**  
  Defines the `Distance` class and `DistanceUnit` enum for representing and converting between meters, centimeters, millimeters, and other distance units.

- **Angle.h**  
  Defines the `Angle` class and `AngleUnit` enum for representing and converting between degrees, radians, and gradians.

- **Speed.h**  
  Defines the `Speed` class for representing linear speed in various units (e.g., m/s, km/h).

- **Time.h**  
  Defines the `Time` class for representing durations and time points in seconds, milliseconds, etc.

- **Distance3D**  
  Represents a 3D distance or position vector with unit support. Provides methods to retrieve each component in any supported unit.

- **Speed3D / Velocity3D**  
  Represents a 3D speed or velocity vector with unit support. Provides methods to retrieve each component in any supported unit.

- **AngularVelocity3D**  
  Represents a 3D angular velocity vector with unit support. Provides methods to retrieve each component in any supported unit.

## Typical Usage

- Represent and convert distances, angles, speeds, and times in a type-safe way.
- Prevent unit mismatches and bugs in calculations.
- Integrate with coordinate, mapping, and planning modules for robust robotics code.

## Example

```cpp
#include <TinyRobotics.h>

Distance d1(100, DistanceUnit::CM); // 100 cm
float meters = d1.getValue(DistanceUnit::M);   // Convert to meters

Angle a1(90, AngleUnit::DEG);       // 90 degrees
float radians = a1.getValue(AngleUnit::RAD);      // Convert to radians

Distance3D pos(1, 2, 3, DistanceUnit::M); // 3D position in meters
float xInCm = pos.getX(DistanceUnit::CM); // Convert X component to centimeters

Speed3D vel(10, 20, 30, SpeedUnit::KMH); // 3D velocity in km/h
float zInMS = vel.getZ(SpeedUnit::MPS); // Convert Z component to m/s

AngularVelocity3D angVel(45, 90, 180, AngularVelocityUnit::DEG_S); // 3D angular velocity in degrees per second
float yInRadS = angVel.getY(AngularVelocityUnit::RAD_S); // Convert Y component to rad/s
```

## See Examples

- [Units Example](../../../examples/others/units/units.ino)

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [planning/](../planning/) for path planning and navigation
- [maps/](../maps/) for map and path data structures

---

This module is designed for embedded and desktop robotics, ensuring safe and clear unit handling in all calculations.