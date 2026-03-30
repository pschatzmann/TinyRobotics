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

## Typical Usage

- Represent and convert distances, angles, speeds, and times in a type-safe way.
- Prevent unit mismatches and bugs in calculations.
- Integrate with coordinate, mapping, and planning modules for robust robotics code.

## Example

```cpp
#include <TinyRobotics.h>

Distance d1(100, DistanceUnit::CM); // 100 cm
float meters = d1.getDistance(DistanceUnit::M);   // Convert to meters

Angle a1(90, AngleUnit::DEG);       // 90 degrees
float radians = a1.getAngle(AngleUnit::RAD);      // Convert to radians
```

## See Examples

- [Units Example](../../../examples/others/units/units.ino)

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [planning/](../planning/) for path planning and navigation
- [maps/](../maps/) for map and path data structures

---

This module is designed for embedded and desktop robotics, ensuring safe and clear unit handling in all calculations.