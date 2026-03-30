# TinyRobotics Motors Module

## Overview
This module provides classes and utilities for controlling DC motors (via H-Bridge drivers) and standard RC servo motors. It is designed for embedded and Arduino environments.

## Features

## Features
- Support for DC motors using H-Bridge drivers (e.g., L298N, L293D, TB6612FNG)
- Support for standard RC servo motors
- PWM and direction control for H-Bridge
- Angle and pulse width control for servos
- Motor base class for extensibility

## Key Classes
- `Motor`: Abstract base class for all motor types
- `HBridge`: High-level H-Bridge DC motor driver
- `ServoMotor`: High-level wrapper for Arduino Servo control
- `Motors`: Utility for managing multiple motors

## Example Usage
```cpp
// Example: Control a DC motor with H-Bridge
#include <TinyRobotics.h>

tinyrobotics::HBridge motor(5, 6, 9); // IN1=5, IN2=6, PWM=9

void setup() {
	motor.setSpeed(128); // Half speed forward
	motor.setSpeed(-255); // Full speed reverse
	motor.setSpeedPercent(50); // 50% forward
	motor.stop(); // Brake
}

void loop() {
	// Your control logic here
}

// Example: Control a Servo motor
#include <TinyRobotics.h>

tinyrobotics::ServoMotor servo;

void setup() {
	servo.attach(9);           // Attach to pin 9
	servo.setConstraints(-45, 45); // Limit range to -45..45 degrees
	servo.setAngle(45);        // Set to 45 degrees left
	int angle = servo.getAngle(); // Read last angle
	servo.detach();            // Detach when done
}

void loop() {
	// Your control logic here
}

```
## Dependencies

- [ESP32Servo](https://github.com/jkb-git/ESP32Servo)
- [Servo](https://docs.arduino.cc/libraries/servo/)

## See Also

- [Control Module](../control/README.md)
- [Vehicles Module](../vehicles/README.md)

---
