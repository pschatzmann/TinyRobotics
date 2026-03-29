/**
 * @file WheelEncoder.ino
 * @brief Example: Basic usage of WheelEncoder with interrupt-driven tick updates.
 *
 * Demonstrates how to use the TinyRobotics WheelEncoder class to measure wheel
 * rotation, speed, and distance using hardware interrupts. The encoder reports
 * messages at a configurable interval, which can be printed or processed by
 * adding a MessageHandler.
 *
 * - Set your wheel diameter and ticks per revolution for accurate measurement.
 * - Attach the interrupt to your encoder pin (see setup).
 * - Output is printed to Serial via MessageHandlerPrint.
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

WheelEncoder encoder;
MessageHandlerPrint printer(Serial); // print info on Serial
const int pin = 4 

void IRAM_ATTR onEncoderTick() {
  encoder.setTick();
}

void setup() {
  Serial.begin(115200);

  encoder.setWheelDiameter(0.065); // 65mm wheel
  encoder.setTicksPerRevolution(20);
  encoder.setReportingFrequencyMs(1000); // Report every 1 second
  encoder.subscribe(printer);
  encoder.begin();
  // Attach interrupt to your encoder pin (example: GPIO 4)
  attachInterrupt(digitalPinToInterrupt(pin), onEncoderTick, RISING);
}

void loop() {
  // In a real application, you would likely want to do this less frequently
  // float distance = encoder.getDistanceM();
  // float speed = encoder.getSpeedMPS();
  // Serial.print("Distance (m): ");
  // Serial.print(distance);
  // Serial.print("\tSpeed (m/s): ");
  // Serial.println(speed);
  // delay(500);
}
