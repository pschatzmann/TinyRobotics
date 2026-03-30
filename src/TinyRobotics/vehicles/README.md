# Vehicles Module

This module provides high-level abstractions for controlling various types of robotic vehicles using the TinyRobotics library. Each vehicle class encapsulates the logic for its drive and steering mechanisms, making it easy to control different vehicle types with a unified interface.

## Supported Vehicles

- **Car4WD**: Four-wheel drive car with direction control by adjusting the speed of four motors (no steering servo).
- **CarAckerman**: Car with Ackerman steering (front wheel steering via servo) and a single drive motor.
- **Quadrotor**: Quadcopter with four independently controlled motors for throttle, roll, pitch, and yaw.
- **AirPlane**: Fixed-wing airplane with motor (throttle), rudder, elevator, and ailerons.
- **MotorBoat**: Boat with a single drive motor and a rudder for steering.

## Example Usage

```cpp
#include <TinyRobots.h>

// Car4WD
CarDifferential<4> car;
car.setPins(0, 5, 6, 9); // front left motor
car.setPins(1, 7, 8, 10); // front right motor
car.setPins(2, 11, 12, 13); // rear left motor
car.setPins(3, 14, 15, 16); // rear right motor
car.setSpeed(60);
car.setTurn(30);
car.end();

// CarAckerman
CarAckerman ack;
ack.setPins(5, 6, 9, 10); // drive motor and steering servo
ack.setSpeed(70);
ack.setSteering(20);
ack.end();

// Quadrotor
Quadrotor quad;
quad.setPins(0, 5, 6, 9);   // front left motor
quad.setPins(1, 7, 8, 10);  // front right motor
quad.setPins(2, 11, 12, 13); // rear left motor
quad.setPins(3, 14, 15, 16); // rear right motor
quad.setThrottle(50);
quad.setRoll(10);
quad.setPitch(-5);
quad.setYaw(15);
quad.end();

// AirPlane
AirPlane plane;
plane.setPinsMotor(5, 6, 9);      // HBridge pins
plane.setPinRudder(10);                         // rudder servo
plane.setPinElevator(11);                     // elevator servo
plane.setPinsAilerons(12, 13); // aileron servos
plane.setThrottle(70);
plane.setRudder(20);
plane.setElevator(-10);
plane.setAilerons(15, -15);
plane.end();

// MotorBoat
MotorBoat boat;
boat.setPins(5, 6, 9, 10);
boat.setThrottle(80);
boat.setRudder(25);
boat.end();
```

## File Overview

- `Car4WD.h`      — Four-wheel drive car
- `CarAckerman.h` — Ackerman steering car
- `Quadrotor.h`   — Quadcopter
- `AirPlane.h`    — Fixed-wing airplane
- `MotorBoat.h`   — Motor boat

## Notes

The motors are defined as templates, so you can switch out the motor type or implement your own motor implementation:

```cpp
Quadrotor<BrushedMotor> quadBushed;
Quadrotor<BrushlessMotor> quadBrushless;
```

## See Also

- [motors/README.md](../motors/README.md)
- [sensors/README.md](../sensors/README.md)
- [maps/README.md](../maps/README.md)
- [planning/README.md](../planning/README.md)
