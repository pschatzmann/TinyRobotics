
# TinyRobotics Odometry Module

This module provides a flexible, extensible framework for estimating the position and orientation (pose) of a robot in 2D or 3D space using odometry. Odometry is essential for mobile robots to track their movement over time based on wheel encoders, velocity commands, or steering data. The architecture supports modular kinematic models, message-driven integration, and is suitable for both simple and advanced robots.

## Key Concepts

- **Odometry2D**: Tracks 2D position and orientation using modular heading models and speed sources. Supports Ackermann, differential drive, and custom kinematics.
- **OdometryModel2D / OdometryDifferentialDriveModel**: Pluggable models implementing the `IOdometryModel2D` interface for heading (theta) integration.
- **WheelEncoder**: Multi-wheel, vectorized encoder for per-wheel distance and speed measurement, modular and ISpeedSource-compliant.
- **SpeedFromThrottle**: Estimates speed from throttle percentage using calibration data.
- **Message-driven**: Odometry can subscribe to vehicle/motor messages for speed and steering updates.
- **Extensible**: Add your own heading models by implementing `IOdometryModel2D`.

### 3D Odometry Key Concepts
- **Odometry3D**: Tracks 3D position and orientation (yaw, pitch, roll) using modular 3D kinematic models. Suitable for drones, airplanes, and underwater robots.
- **IOdometryModel3D**: Interface for 3D kinematic models. Implement this to define custom 3D vehicle kinematics for use with Odometry3D.
- **AirplaneOdometryModel3D**: Implements IOdometryModel3D for fixed-wing airplanes using throttle and control surface inputs (aileron, elevator, rudder).
- **DroneOdometryModel3D**: Implements IOdometryModel3D for quadcopters/drones using per-motor percentage inputs. Maps motor outputs to 3D velocities and angular rates.

---

## Odometry2D

Tracks the 2D position (x, y) and orientation (theta) of a robot using velocity and steering angle inputs. The kinematic model is injected via the constructor, allowing for Ackermann, differential drive, or custom models.

**Features:**
- Integrates speed and steering angle to estimate robot pose
- Modular: accepts any `IOdometryModel2D` implementation
- Message-driven: can subscribe to vehicle/motor messages for updates
- Provides access to position, orientation, velocity, distance traveled, and last update delta
- Allows resetting and setting the odometry state

**Key Methods:**
- `Odometry2D(MessageSource& vehicle, ISpeedSource& speedSource, IOdometryModel2D& model)`: Constructor
- `begin(initialPosition, initialTheta, wheelBase)`: Initialize odometry state
- `update()`: Integrates pose using the latest speed/steering (set via setters or messages)
- `getPosition()`, `getTheta()`, `getLinearVelocity()`, `getAngularVelocity()`, `getTotalDistance()`, `getLastDelta()`
- `setState()`, `reset()`

**Example (Ackermann):**
```cpp
#include <TinyRobotics.h>
CarAckerman car;
Distance wheelBase(0.3f, DistanceUnit::M);
OdometryModel2D odomModel(wheelBase);
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

### IOdometryModel2D (Interface)
Defines the interface for heading (theta) integration models. Implement this to add custom kinematics and is automatically updated by Odometry2D.

**Key Methods:**
- `computeDeltaTheta(uint16_t deltaTimeMs)`: Returns heading change for the time interval
- `setSpeed(Speed speed)`
- `setSpeed(Speed left, Speed right)`
- `setSteeringAngle(Angle angle)`

### OdometryModel2D (Ackermann/Boat/Default)
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

- **Message Integration:** Odometry2D and Odometry3D can subscribe to messages from vehicles/motors for automatic speed, steering, or control updates.
- **Custom Models:** Implement `IOdometryModel2D` or `IOdometryModel3D` for holonomic, omnidirectional, airplane, drone, or other robot types.

---

## 3D Odometry and Kinematic Models

### Odometry3D
Tracks the 3D position (x, y, z) and orientation (yaw, pitch, roll) of a robot using linear and angular velocity inputs. Suitable for drones, airplanes, underwater vehicles, and robots operating in 3D environments.

**Features:**
- Integrates 3D linear and angular velocities to estimate robot pose
- Uses an `Orientation3D` object for modular orientation handling
- Pluggable kinematic model via `IOdometryModel3D` (see below)
- Message-driven: can subscribe to control/velocity messages for updates
- Provides access to position, orientation, distance traveled, and last update delta
- Allows resetting and setting the odometry state

**Key Methods:**
- `Odometry3D(MessageSource& vehicle, IOdometryModel3D& model)`: Constructor with message-driven integration
- `begin(initialPosition, initialOrientation)`: Initialize odometry state with position and orientation
- `update()`: Integrates pose using the latest velocities from the model
- `getPosition()`: Get current position (x, y, z)
- `getOrientation()`: Get current orientation as `Orientation3D`
- `getTotalDistance()`: Get total distance traveled
- `getLastDelta()`: Get last (dx, dy, dz) update

**Example:**
```cpp
#include <TinyRobotics.h>

DroneOdometryModel3D droneModel;
Odometry3D odom3d(vehicle, droneModel);
void setup() {
    Coordinate<float> startPos(0, 0, 0);
    Orientation3D startOri(0.0f, 0.0f, 0.0f); // yaw, pitch, roll in radians
    odom3d.begin(startPos, startOri);
}
void loop() {
    // Update odometry (message-driven or direct input)
    odom3d.update();
    auto pos = odom3d.getPosition();
    auto ori = odom3d.getOrientation();
    Serial.printf("x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, pitch=%.2f, roll=%.2f\n",
        pos.x, pos.y, pos.z, ori.yaw, ori.pitch, ori.roll);
}
```

### IOdometryModel3D (Interface)
Defines the interface for 3D kinematic models. Implement this to add custom 3D kinematics for drones, airplanes, or other vehicles. Used by `Odometry3D` for pose integration.

**Key Methods:**
- `getLinearVelocity(float& vx, float& vy, float& vz)`: Returns current linear velocity (m/s)
- `getAngularVelocity(float& wx, float& wy, float& wz)`: Returns current angular velocity (rad/s)
- `registerCallback(void (*callback)(void*), void* userData)`: Register a callback for input changes (optional)

---

### AirplaneOdometryModel3D
Implements `IOdometryModel3D` for fixed-wing airplanes using throttle and control surface inputs (aileron, elevator, rudder).

**Features:**
- Maps throttle (0–100%) and control surface deflections (degrees) to body-frame velocities
- Supports message-driven control (handles `MessageContent::Throttle`, `::Roll`, `::Pitch`, `::Yaw`)
- All input values are clamped to safe ranges
- Suitable for simulation, estimation, or as a reference model for real-time control

**Key Methods:**
- `setThrottle(float percent)`: Set throttle (0–100%)
- `setAileron(float deg)`, `setElevator(float deg)`, `setRudder(float deg)`: Set control surfaces (degrees)
- `onMessage(const Message<float>& msg)`: Handle incoming control messages
- `registerCallback(void (*callback)(void*), void* userData)`: Register callback for input changes


### DroneOdometryModel3D
Implements `IOdometryModel3D` for quadcopters/drones using per-motor percentage inputs.

**Features:**
- Maps motor percentages (0–100%) for four motors to body-frame velocities
- Supports message-driven control (handles `MessageContent::MotorSpeed` with `origin_id` 0..3)
- All input values are clamped to 0–100%
- Suitable for simulation, estimation, or as a reference model for real-time control

**Key Methods:**
- `setMotorPercent(int motor, float percent)`: Set percentage for a given motor (0..3)
- `onMessage(const Message<float>& msg)`: Handle incoming motor speed messages
- `registerCallback(void (*callback)(void*), void* userData)`: Register callback for input changes

## Class Documentation

- [odometry](https://pschatzmann.github.io/TinyRobotics/group__odometry.html)

## See Also

- `CarAckerman`, `GenericMotor`, `MotionController2D`, `Frame2D`, `Coordinate`, `Angle`, `Speed`, `Distance`, `Orientation3D`
- Example sketches in the `examples/` folder for real-world usage patterns.
