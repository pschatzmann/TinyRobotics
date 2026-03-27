// Example: Basic usage of WheelEncoder with interrupt-driven tick updates
#include <TinyRobotics.h>

WheelEncoder encoder;
MessageHandlerPrint printer(Serial); // print info on Serial

void IRAM_ATTR onEncoderTick() {
  encoder.setTick();
}

void setup() {
  Serial.begin(115200);

  encoder.setWheelDiameter(0.065); // 65mm wheel
  encoder.setTicksPerRevolution(20);
  encoder.setReportingFrequencyMs(1000); // Report every 1 second
  encoder.addMessageHandler(printer);
  encoder.begin();
  // Attach interrupt to your encoder pin (example: GPIO 4)
  attachInterrupt(digitalPinToInterrupt(4), onEncoderTick, RISING);
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
