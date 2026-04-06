#/**
 * @file std-motors.ino
 * @brief Example sketch demonstrating usage of all supported motor types in TinyRobotics.
 *
 * This example shows how to initialize and control different types of motors:
 * - Brushed DC Motor (H-Bridge)
 * - Brushless DC Motor (ESC)
 * - RC Servo Motor
 * - Stepper Motor
 *
 * Connections:
 * - Update the pin numbers as needed for your hardware setup.
 *
 * @author TinyRobotics Contributors
 * @date 2026-04-01
 */
// TinyRobotics: Example sketch demonstrating all supported motor types
#include <TinyRobotics.h>

// Brushed DC Motor (H-Bridge)
BrushedMotor brushed(0);
// Brushless DC Motor (ESC)
BrushlessMotor brushless(1);
// RC Servo Motor
ServoMotor servo(2);
#if USE_FASTACCEL_STEPPER
// Stepper Motor (using FastAccelStepper)
StepperMotor stepper(3);
#endif

void setup() {
  Serial.begin(115200);
  // Brushed Motor
  brushed.setPins(5, 6);  // Example pins
  brushed.begin();
  brushed.setValuePercent(50); // Percent

  // Brushless Motor
  brushless.setPin(10);  // Example pin
  brushless.begin();
  brushless.setValuePercent(50); // Percent

  // Servo Motor
  servo.setPin(11);  // Example pin
  servo.begin();
  servo.setAngle(45); // Set servo to 45 degrees to the left
  //servo.setValuePercent(50); // Move to 50% (90 degrees) - alternative to setAngle

#if USE_FASTACCEL_STEPPER
  // Stepper Motor
  stepper.setPins(2, 3, 4); // Example pins: step, dir, enable
  stepper.setMaxSpeed(1000); // steps/sec
  stepper.setStepsPerRevolution(200); // typical 1.8 degree stepper
  stepper.setAccelerationMs(2000); // ms to reach max speed
  stepper.begin();
  stepper.setValuePercent(50); // Move at 50% of max speed (continuous)
#endif
}

void loop() {
}
