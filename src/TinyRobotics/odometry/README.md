
# TinyRobotics Odometry Module

This module provides a flexible, extensible framework for estimating the position and orientation (pose) of a robot in 2D or 3D space using odometry. Odometry is essential for mobile robots to track their movement over time based on wheel encoders, velocity commands, or steering data. The architecture supports modular kinematic models, message-driven integration, and is suitable for both simple and advanced robots.

## Key Concepts

- **Odometry2D**: Tracks 2D position and orientation using modular heading models and speed sources. Supports Ackermann, differential drive, and custom kinematics.
- **OdometryHeadingModel / OdometryDifferentialDriveModel**: Pluggable models implementing the `IOdometryHeadingModel2D` interface for heading (theta) integration.
- **WheelEncoder**: Multi-wheel, vectorized encoder for per-wheel distance and speed measurement, modular and ISpeedSource-compliant.
- **SpeedFromThrottle**: Estimates speed from throttle percentage using calibration data.
- **Message-driven**: Odometry can subscribe to vehicle/motor messages for speed and steering updates.
- **Extensible**: Add your own heading models by implementing `IOdometryHeadingModel2D`.

---

## Odometry2D

Tracks the 2D position (x, y) and orientation (theta) of a robot using velocity and steering angle inputs. The kinematic model is injected via the constructor, allowing for Ackermann, differential drive, or custom models.

**Features:**
- Integrates speed and steering angle to estimate robot pose
- Modular: accepts any `IOdometryHeadingModel2D` implementation
- Message-driven: can subscribe to vehicle/motor messages for updates
- Provides access to position, orientation, velocity, distance traveled, and last update delta
- Allows resetting and setting the odometry state

**Key Methods:**
- `Odometry2D(MessageSource& vehicle, ISpeedSource& speedSource, IOdometryHeadingModel2D& model)`: Constructor
- `begin(initialPosition, initialTheta, wheelBase)`: Initialize odometry state
- `update()`: Integrates pose using the latest speed/steering (set via setters or messages)
- `getPosition()`, `getTheta()`, `getLinearVelocity()`, `getAngularVelocity()`, `getTotalDistance()`, `getLastDelta()`
- `setState()`, `reset()`

**Example (Ackermann):**
```cpp
#include <TinyRobotics.h>
CarAckerman car;
Distance wheelBase(0.3f, DistanceUnit::M);
OdometryHeadingModel odomModel(wheelBase);
SpeedFromThrottle speedEstimator(Speed(5, SpeedUnit::KPH));
Odometry2D odometry(car, speedEstimator, odomModel);

void loop() {
	odometry.update();
	auto pos = odometry.getPosition();
	float heading = odometry.getTheta();
	Serial.printf("x=%.2f, y=%.2f, theta=%.2f\n", pos.x, pos.y, heading);
}


**Example (Differential Drive):**
```cpp
#include <TinyRobotics.h>
Distance wheelBase(0.2f, DistanceUnit::M);
OdometryDifferentialDriveModel diffModel(wheelBase);
Odometry2D odometry(vehicle, leftEncoder, diffModel);

void loop() {
	odometry.update();
}
```

---

## Modular Heading Models

### IOdometryHeadingModel2D (Interface)
Defines the interface for heading (theta) integration models. Implement this to add custom kinematics and is automatically updated by Odometry2D.

**Key Methods:**
- `computeDeltaTheta(uint16_t deltaTimeMs)`: Returns heading change for the time interval
- `setSpeed(Speed speed)`
- `setSpeed(Speed left, Speed right)`
- `setSteeringAngle(Angle angle)`

### OdometryHeadingModel (Ackermann/Boat/Default)
Implements Ackermann and default (steering as angular velocity) kinematics. Use for car-like robots or boats.

### OdometryDifferentialDriveModel
Implements differential drive kinematics using left/right wheel speeds.

---


## WheelEncoder

The `WheelEncoder` class provides robust, multi-wheel support for measuring wheel rotation and computing per-wheel distance and speed using encoder ticks. It is designed for modular integration with odometry pipelines and supports any number of wheels (differential, skid-steer, etc.).

**Features:**
- Supports any number of wheels (configurable at construction)
- Vectorized state for distance, speed, and tick timing per wheel
- Interface-compliant with `ISpeedSource` for modular odometry integration
- Configurable wheel diameter and ticks per revolution for accurate distance estimation
- Periodic reporting of distance and speed via the `MessageSource` interface
- Slip calibration support to compensate for wheel slip or surface effects
- Designed for use with interrupt-driven tick updates (call `setTick(motor)` in your ISR)

**Key Methods:**
- `WheelEncoder(size_t numWheels = 1)`: Construct for N wheels
- `setWheelDiameter(Distance diameter)` / `setWheelDiameter(float, DistanceUnit)`
- `setTicksPerRevolution(int ticks)`
- `begin()`: Reset and start reporting
- `setTick(size_t motor = 0)`: Call in your encoder ISR for each wheel
- `getDistanceM(size_t motor = 0)`, `getSpeedMPS(size_t motor = 0)`, `getDistance(DistanceUnit, size_t motor = 0)`
- `setSlipFactor(float)`, `calibrateSlip(float actualDistanceM)`

**Example (Differential Drive):**
```cpp
#include <TinyRobotics.h>
WheelEncoder encoder(2); // Two wheels
encoder.setWheelDiameter(0.065); // 65mm wheel
encoder.setTicksPerRevolution(20);
encoder.begin();
// In your interrupt:
encoder.setTick(0); // Left wheel
encoder.setTick(1); // Right wheel
// In your main loop:
float leftDistance = encoder.getDistanceM(0);
float rightDistance = encoder.getDistanceM(1);
float leftSpeed = encoder.getSpeedMPS(0);
float rightSpeed = encoder.getSpeedMPS(1);
```

This class is intended for embedded robotics applications (Arduino, ESP32, etc.) and integrates with the TinyRobotics messaging framework. It is suitable for use as a modular speed/distance source in extensible odometry pipelines.

---

## SpeedFromThrottle

Estimates vehicle speed from throttle percentage using calibration data. Useful when direct speed measurement is unavailable or unreliable.

**Features:**
- Piecewise linear mapping from throttle percent (-100% to 100%) to speed (m/s)
- User-provided calibration points for flexible, non-linear mapping
- Default: 0% throttle = 0 m/s, 100% = maxSpeed, -100% = -maxSpeed
- Supports both direct speed queries and integration with the TinyRobotics message system
- Can be used as a MessageHandler (receives Throttle, emits Speed)

**Key Methods:**
- `SpeedFromThrottle(maxSpeedMps)`
- `addSpeedCalibration(throttle, speed)`
- `getSpeed(throttle)`

---

## Usage Notes

- For Ackermann vehicles, set the wheelbase using the model constructor.
- For differential drive, use `OdometryDifferentialDriveModel` and provide left/right speeds.
- All angles are in radians; all distances in meters.
- The module is designed for extensibility—implement your own heading model for custom kinematics.

---

## Advanced Topics

- **Message Integration:** Odometry2D can subscribe to messages from vehicles/motors for automatic speed/steering updates.
- **Custom Models:** Implement `IOdometryHeadingModel2D` for holonomic, omnidirectional, or other robot types.
- **3D Odometry:** See `Odometry3D` for drones, underwater, or aerial robots (not covered in detail here).

---

## See Also

- `CarAckerman`, `GenericMotor`, `MotionController2D`, `Frame2D`, `Coordinate`, `Angle`, `Speed`, `Distance`
- Example sketches in the `examples/` folder for real-world usage patterns.

### Odometry3D
Tracks the 3D position (x, y, z) and orientation (yaw, pitch, roll) of a robot using linear and angular velocity inputs. Suitable for drones, underwater vehicles, and robots operating in 3D environments.

**Features:**
- Integrates 3D linear and angular velocities to estimate robot pose
- Uses an Orientation3D object for modular orientation handling
- Provides access to position, orientation, distance traveled, and last update delta
- Allows resetting and setting the odometry state

**Key Methods:**
- `begin(initialPosition, initialOrientation)`: Initialize odometry state with position and orientation
- `update(vx, vy, vz, wx, wy, wz, deltaTimeMs)`: Update pose with new velocities (optionally with time delta)
- `update(vx, vy, vz, wx, wy, wz)`: Update pose using automatic time delta (uses millis())
- `getPosition()`: Get current position (x, y, z)
- `getOrientation()`: Get current orientation as Orientation3D
- `getTotalDistance()`: Get total distance traveled
- `getLastDelta()`: Get last (dx, dy, dz) update

**Example:**
```cpp
#include <TinyRobotics.h>
using namespace tinyrobotics;

Odometry3D odom3d;
void setup() {
	Coordinate<float> startPos(0, 0, 0);
	Orientation3D startOri(0.0f, 0.0f, 0.0f); // yaw, pitch, roll in radians
	odom3d.begin(startPos, startOri);
}
void loop() {
	// Example velocities (vx, vy, vz, wx, wy, wz)
	odom3d.update(0.1, 0, 0, 0, 0, 0); // Move forward
	auto pos = odom3d.getPosition();
	auto ori = odom3d.getOrientation();
	Serial.printf("x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, pitch=%.2f, roll=%.2f\n",
		pos.x, pos.y, pos.z, ori.yaw, ori.pitch, ori.roll);
}
```
