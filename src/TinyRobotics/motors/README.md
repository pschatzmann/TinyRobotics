# Motor Drivers Overview

This folder contains all subclasses of the abstract `Motor` base class in TinyRobotics. Each driver provides a unified interface for initialization, shutdown, and pin configuration, but targets different types of motors:

## Dependencies

- **FastAccelStepper**: Required for Stepper motor support. Install from the Arduino Library Manager or [GitHub](https://github.com/ESP32DIYer/FastAccelStepper).
- **Servo** or **ESP32Servo**: Required for BrushlessMotor and ServoMotor. Install from the Arduino Library Manager or [ESP32Servo GitHub](https://github.com/jkb-git/ESP32Servo) for ESP32.

## Subclasses of Motor

### 1 Brushed Motors / HBridge
- **File:** BrushedMotor.h
- **Description:** High-level H-Bridge motor driver for bidirectional DC motors (e.g., L298N, L293D, TB6612FNG).
- **Features:** Forward/reverse/brake, PWM speed control, percent speed, optional PWM frequency, speed limiting.
- **Typical Use:** DC motors with H-Bridge drivers.

**Minimal Example:**
```cpp
#include <TinyRobotics.h>
BrushedMotor motor;
motor.setPins(5, 6, 9); // IN1, IN2, PWM
motor.begin();
motor.setSpeed(50); // 50% forward
motor.end();
```

### 2. Stepper
- **File:** Stepper.h
- **Description:** Stepper motor driver using FastAccelStepper for precise control.
- **Features:** Continuous speed mode, move by revolutions, move by distance, acceleration, callback support.
- **Typical Use:** Stepper motors for robotics and automation.

**Minimal Example:**
```cpp
#include <TinyRobotics.h>
Stepper stepper;
stepper.setPins(2, 3, 4); // STEP, DIR, ENABLE
stepper.setMaxSpeed(1000);
stepper.setStepsPerRevolution(200);
stepper.setAccelerationMs(2000);
stepper.begin();
stepper.setSpeed(50); // 50% speed
stepper.end();
```

### 3. BrushlessMotor
- **File:** BrushlessMotor.h
- **Description:** Brushless DC motor (ESC) driver using Servo/ESP32Servo library.
- **Features:** Attach to pin, set speed as percentage, detach, simple throttle control.
- **Typical Use:** Brushless motors with ESCs (e.g., drones, RC cars).

**Minimal Example:**
```cpp
#include <TinyRobotics.h>
BrushlessMotor motor;
motor.setPin(9);
motor.begin();
motor.setSpeed(50); // 50% throttle
motor.end();
```

### 4. ServoMotor
- **File:** Servo.h
- **Description:** High-level wrapper for standard RC servo motors using the Servo library.
- **Features:** Attach/detach, set angle (degrees), set by pulse width, constrain angle range, read last angle.
- **Typical Use:** Standard RC servos for steering, arms, etc.

**Minimal Example:**
```cpp
#include <TinyRobotics.h>
ServoMotor servo;
servo.setPin(9);
servo.begin();
servo.setAngle(45); // 45 degrees
servo.end();
```

---

All these classes inherit from the abstract `Motor` base class, which defines a common interface for initialization, shutdown, and pin configuration.
