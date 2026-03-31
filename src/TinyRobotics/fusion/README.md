
# Sensor Fusion

**Fusion2D** is a header-only 2D sensor fusion EKF for mobile robots.
It fuses **position, heading, and speed** from multiple optional sensors:

* IMU (gyroscope + optional acceleration)
* Wheel encoder (speed)
* Absolute heading (magnetometer or GPS-derived)
* GPS position

The class is designed to be **sensor-agnostic**: any combination of sensors can be used.

---

## Features

* Fully 2D EKF with position, heading, speed, and gyro bias estimation.
* Optional IMU acceleration used for speed estimation when wheel encoder is missing.
* Optional wheel speed input.
* Optional absolute heading input to correct gyro drift.
* Optional GPS input to correct position.
* Handles missing sensors gracefully.
* Header-only, easy to integrate into embedded or desktop projects.

---

## Usage

### Include

```cpp
#include "TinyRobotics.h"
```

### Create the Fusion2D instance

```cpp
Fusion2D fusion;
```

### Set initial state

```cpp
fusion.begin(Coordinate(10.0f, 5.0f), Angle(45, AngleUnit::DEG)); // x=10m, y=5m, heading=45 degrees
```

### Sensor updates

```cpp
// 1️⃣ IMU (gyro + optional acceleration)
fusion.updateIMU(timeMs, gyroZ, ax, ay);

// 2️⃣ Wheel speed (optional)
fusion.updateSpeed(timeMs, wheelSpeed);

// 3️⃣ Absolute heading (magnetometer or GPS-derived)
fusion.updateHeading(timeMs, heading);

// 4️⃣ GPS position (optional)
fusion.updateGPS(timeMs, x, y, accuracy);
```

### Prediction

* `predict(timeMs)` is automatically called by update methods.
* You can call it manually if you want to propagate the state between sensor updates:

```cpp
fusion.predict(timeMs, ax, ay); // optional IMU acceleration if wheel speed is missing
```

### Get fused state

```cpp
Coordinate pos = fusion.getPosition();
Speed speed = fusion.getSpeed();
Angle heading = fusion.getHeading();
```

---

## Notes

* **IMU acceleration (`ax`, `ay`)** is optional. Units: m/s². Only used if wheel speed is missing.
* **Gyro (`yawRate`)** is used for heading prediction. Units: rad/s.
* **Wheel speed** overrides IMU acceleration for velocity integration.
* **Heading** corrects the gyro drift.
* **GPS** corrects position independently.
* Can run with any subset of sensors.

---

## Example Initialization

```cpp
Fusion2D fusion;
fusion.reset(0);                     // reset covariance and time
fusion.setState(0.0f, 0.0f, 0.0f);  // start at origin with heading=0
```

