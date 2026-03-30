# TinyRobotics Odometry Module

This module provides classes for estimating the position and orientation (pose) of a robot in 2D space using odometry. Odometry is essential for mobile robots to track their movement over time based on wheel encoders, velocity commands, or steering data.

## Classes

### Odometry2D
Tracks the 2D position (x, y) and orientation (theta) of a robot using velocity and steering angle inputs. Supports both differential drive and Ackermann steering kinematics.

**Features:**
- Integrates speed and steering angle to estimate robot pose
- Supports both differential drive and Ackermann vehicles
- Automatically selects kinematic model based on wheelbase
- Provides access to position, orientation, velocity, distance traveled, and last update delta
- Allows resetting and setting the odometry state

**Key Methods:**
- `begin(initialPosition, initialTheta, wheelBase)`: Initialize odometry state and optionally set wheelbase for Ackermann
- `update(speed, steeringAngle, deltaTimeMs)`: Update pose with new speed and steering angle (optionally with time delta)
- `getPosition()`: Get current position (x, y)
- `getTheta()`: Get current orientation (radians)
- `getLinearVelocity()`, `getAngularVelocity()`: Get current velocities
- `getTotalDistance()`: Get total distance traveled
- `getLastDelta()`: Get last (dx, dy, dtheta) update
- `reset()`, `setState()`: Reset or set odometry state

**Kinematic Models:**
- **Differential Drive:** Uses angular velocity as steering input
- **Ackermann:** Uses wheelbase and steering angle to compute angular velocity

**Example:**
```cpp
Odometry2D odom;
odom.begin(Coordinate<DistanceM>(0, 0), 0.0f, Distance(0.25, DistanceUnit::M)); // 25cm wheelbase
// In your control loop:
odom.update(currentSpeed, currentSteeringAngle);
auto pos = odom.getPosition();
float heading = odom.getTheta();
Serial.printf("x=%.2f, y=%.2f, theta=%.2f\n", pos.x, pos.y, heading);
```

---

### SpeedFromThrottle
Estimates vehicle speed from throttle percentage using calibration data. Useful when direct speed measurement is unavailable or unreliable.

**Features:**
- Piecewise linear mapping from throttle percent (-100% to 100%) to speed (m/s)
- User-provided calibration points for flexible, non-linear mapping
- Default: 0% throttle = 0 m/s, 100% = maxSpeed, -100% = -maxSpeed
- Supports both direct speed queries and integration with the TinyRobotics message system
- Can be used as a MessageHandler (receives Throttle, emits Speed)

**Key Methods:**
- `SpeedFromThrottle(maxSpeedMps)`: Constructor, sets max speed for 100% throttle
- `addSpeedCalibration(throttle, speed)`: Add or update a calibration point
- `getSpeedMPS(throttle)`: Get interpolated speed in m/s for a given throttle
- `getSpeed(throttle)`: Get a Speed object for a given throttle

**Example:**
```cpp
SpeedFromThrottle speedMap(2.0f); // 2 m/s at 100% throttle
speedMap.addSpeedCalibration(50.0f, 1.0f); // 1 m/s at 50%
float speed = speedMap.getSpeedMPS(75.0f); // Interpolated speed
// As a message handler/source:
bus.subscribe(speedMap); // Receives Throttle messages, emits Speed messages
```

---

## Usage Notes
- For Ackermann vehicles, set the wheelbase using `begin()` or by assigning to `wheelBase`.
- For differential drive, leave wheelbase at zero (default).
- All angles are in radians; all distances in meters.
- The module is designed for extensibility and can be adapted for more advanced odometry models.

---

## License
See the main project for license information.
