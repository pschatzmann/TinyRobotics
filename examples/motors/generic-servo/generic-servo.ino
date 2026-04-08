#/**
 * @file generic-servo.ino
 * @brief Example: Using GenericMotor to control a standard servo with TinyRobotics.
 *
 * This example demonstrates how to use the GenericMotor abstraction to control
 * a standard hobby servo using a value callback. The servo is swept from -90 to 90 degrees.
 *
 * - Uses Servo or ESP32Servo library depending on platform.
 * - Shows how to map logical angles to servo PWM values.
 *
 * @author TinyRobotics Contributors
 * @date 2026-04-01
 */
#ifdef ESP32
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#include "TinyRobotics.h"

Servo myServo;
GenericMotor<float> motor(0, &myServo);

void setup() {
  Serial.begin(115200);
  // Define value callback to handle -90..90 degree angle control
  motor.setValueCallback([](float value, GenericMotor<float>& motor) {
    motor.getDriver<Servo>().write(map((int)value, -90, 90, 0, 180));
  });
  motor.setBeginCallback([](GenericMotor<float>& motor) {
    myServo.attach(9);  // Attach the servo to pin 9
    return true;        // Return true to indicate successful start
  });
  motor.setEndCallback(
      [](GenericMotor<float>& motor) { motor.getDriver<Servo>().detach(); });

  motor.begin();
}

void loop() {
  // Example: Sweep the servo back and forth
  for (int i = -90; i <= 90; i += 5) {
    motor.setAngle(
        i);  // This will call the value callback to set the servo angle
    delay(500);
  }
  for (int i = 90; i >= -90; i -= 5) {
    motor.setAngle(i);
    delay(500);
  }
}
