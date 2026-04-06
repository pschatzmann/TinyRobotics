# TinyRobotics Motors Module

## Overview

The TinyRobotics Motors module is a flexible and extensible framework for controlling a wide variety of motor types in embedded and Arduino environments. It provides unified abstractions and drivers for:

- **Brushed DC motors** (via H-Bridge drivers such as L298N, L293D, TB6612FNG)
- **Brushless DC motors** (ESC-based, using servo signals)
- **Standard RC servo motors** (using Arduino Servo or ESP32Servo libraries)
- **Stepper motors** (with optional FastAccelStepper integration)
- **Custom and external motor drivers** (via the GenericMotor class and user-defined callbacks)

Key features include:

- Consistent API for all supported motor types
- PWM and direction control for brushed and brushless motors
- Angle and pulse width control for servos and brushless motors
- Stepper motor support with acceleration and speed control
- Easy integration of custom motor drivers without external dependencies
- Modular design for use in robots, vehicles, and automation projects
- Provides template for defining the numeric type for the speed (default float)

The module is designed for portability and can be configured to use or avoid external libraries as needed. All motor drivers inherit from a common base class, making it easy to swap implementations or extend functionality for new hardware.

## Motor Configuration (utils/Config.h)

Motor-specific settings can be adjusted in `src/TinyRobotics/utils/Config.h`:

- `USE_EXTERNAL_MOTOR_LIBRARIES` (default: `true`): Enables use of external libraries for motor control.
- `USE_SERVO_LIBRARY` (default: `USE_EXTERNAL_MOTOR_LIBRARIES`): Use Arduino Servo/ESP32Servo for servo motors.
- `USE_FASTACCEL_STEPPER` (default: `USE_EXTERNAL_MOTOR_LIBRARIES`): Use FastAccelStepper for stepper motors.

You can override these macros before including TinyRobotics to customize which motor drivers are enabled.

Example:

```cpp
#define USE_SERVO_LIBRARY false
#include <TinyRobotics.h>
```

## Motor Classes

- `Motor`: Abstract base class for all motor types. Defines the common interface for all motors.
- `BrushedMotor`: High-level H-Bridge DC motor driver for bidirectional DC motors (L298N, L293D, TB6612FNG, etc.).
- `BrushlessMotor`: Interface for brushless DC motors (ESC-based, uses servo signal).
- `ServoMotor`: High-level wrapper for standard RC servo motors (uses Arduino Servo or ESP32Servo library).
- `StepperMotor`: Interface for stepper motors (uses FastAccelStepper library if enabled).
- `GenericMotor`: Flexible abstraction for integrating custom/external motor drivers using callbacks.

## Example Usage

### DC Motor (H-Bridge)

```cpp
#include <TinyRobotics.h>

BrushedMotor motor(1); // #1

void setup() {
  motor.setPins(5, 6, 9); // IN1=5, IN2=6, PWM=9)
  motor.begin();
  motor.setValuePercent(50); // Half speed forward
  motor.setValuePercent(-50); // Half speed reverse
  motor.end(); // Brake
}

void loop() {
  // Your control logic here
}
```

### Servo Motor

```cpp
#include <TinyRobotics.h>

ServoMotor servo(2); // #2

void setup() {
  servo.setPin(9);           // Attach to pin 9
  servo.begin()
  //servo.setConstraints(-45, 45); // Limit range to -45..45 degrees
  servo.setAngle(45);        // Set to 45 degrees left
  int angle = servo.getAngle(); // Read last angle
  servo.end();            // Detach when done
}

void loop() {
  // Your control logic here
}
```

### Stepper Motor (if enabled)

```cpp
#include <TinyRobotics.h>

StepperMotor stepper;
void setup() {
  stepper.setPins(2, 3, 4);
  stepper.setMaxSpeed(1000);
  stepper.setStepsPerRevolution(200);
  stepper.begin();
  motor.setValuePercent(50); // Half speed forward
  motor.setValuePercent(-50); // Half speed reverse
  stepper.end();
}
```

### Generic Motor (Custom Driver)

```cpp
#include <TinyRobotics.h>

MyMotorDriver driver;
GenericMotor<float> motor(3, &driver); // #3
void setup() {
  motor.setValueCallback([](int8_t value, GenericMotor& m) {
    MyMotorDriver* drv = m.getMotor<MyMotorDriver>();
    drv->setPWM(value);
  });
  // Optional: custom logic for begin and end
  motor.setBeginCallback([](GenericMotor<float>& m) {
    // Custom start logic (e.g., enable power)
    return true;
  });
//   motor.setEndCallback([](GenericMotor<float>& m) {
//     // Custom stop logic (e.g., disable power)
//   });
  motor.begin();
  motor.setValuePercent(50); // Half speed forward
  motor.setValuePercent(-50); // Half speed reverse
  motor.end();
}
```

## Generic Motor Implementation Approaches

You can use a platform specifc motor control library to implement the callbacks. Alternatively you can just the PWM functionality provided by your platform:

### H-Bridge (DC motor control) Logic

- PWM Frequency: ~1 kHz – 20 kHz
- Speed is driven by Duty cycle: 0–100% on EN pins

### Servo Motor Logic

- Fixed PWM frequency: ~50 Hz (20 ms period)
- Pulse width determines position:
  - ~1 ms → 0°
  - ~1.5 ms → 90°
  - ~2 ms → 180°

### Brushed Motor Logic

- Fixed PWM frequency
- Puse width determines speed
- Frequency ranges:
  - 100 – 500 Hz Works, but audible noise
  - 1 – 10 kHz Most common
  - 16 – 25 kHz Above human hearing
  - 25 – 100 kHz Less common, more switching loss

### Stepper Motor Logic (e.g. using L298N driver)

- One pulse is one step, so we can drive the motor speed by the pwm frequency: just keep the duty at 50%.
- Very slow motion: ~1 Hz – 50 Hz
- Normal range: ~100 Hz – 2 kHz
- Common sweet spot: 200 Hz – 1000 Hz
- Fast operation: 2 kHz – 10 kHz

### Stepper Motor Logic (e.g. using L298N driver)
- You need to drive the pins in steps (in1, in2, in3, in4):
  - step 1: 1 0 1 0
  - step 2: 0 1 1 0
  - step 3: 0 1 0 1
  - step 4: 1 0 0 1
- The speed is determined by the speed in which you switch the steps.
- Leave EN pins enabled!



## Dependencies

- [ESP32Servo](https://github.com/jkb-git/ESP32Servo)
- [Servo](https://docs.arduino.cc/libraries/servo/)
- [FastAccelStepper](https://github.com/ESP32DIYer/FastAccelStepper) (for StepperMotor support)

## See Also

- [Control Module](../control/README.md)
- [Vehicles Module](../vehicles/README.md)

---
