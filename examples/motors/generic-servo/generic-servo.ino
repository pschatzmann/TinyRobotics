#ifdef ESP32
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#include "TinyRobotics.h"

Servo myServo;
GenericMotor motor(0, &myServo);

void setup() {
  Serial.begin(115200);
  myServo.attach(9);  // Attach the servo to pin 9
  // Define value callback to handle -90..90 degree angle control
  motor.setValueCallback([](int8_t value, GenericMotor& motor) {
    motor.getMotor<Servo>().write(map(value, -90, 90, 0, 180));
  });
  motor.begin();
}

void loop() {
  // Example: Sweep the servo back and forth
  for (int i = -90; i <= 90; i += 5) {
    motor.setAngle(i);  // This will call the value callback to set the servo angle
    delay(500);
  }
  for (int i = 90; i >= -90; i -= 5) {
    motor.setAngle(i);
    delay(500);
  }
}

