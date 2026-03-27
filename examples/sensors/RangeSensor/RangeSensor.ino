// Example: Basic usage of RangeSensor
#include <TinyRobotics.h>
#include <Ultrasonic.h>  // use the Ultrasonic library for simulating a range sensor (e.g., HC-SR04)
#undef CM

// Define a simple frame hierarchy: world -> base -> lidar
Frame2D world{FrameType::WORLD, 0};
// vehicule is at (10, 20) facing up (90 degrees)
Frame2D base{FrameType::BASE, 0, world, Transform2D(10, 20, 90)};
// LIDAR is 20cm forward and 10 cm to the right of the base, facing the same
// direction as the base
Frame2D lidar{FrameType::LIDAR, 0, base, Transform2D(0.2, -0.1, 0)};
// Frame mgmt
FrameMgr2D tf;
// lidar sensor
Ultrasonic ultrasonic(12, 13);        // Example pins for trigger and echo
RangeSensor sensor(0);                // 0 degrees = forward
MessageHandlerPrint printer(Serial);  // print info on Serial
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  // Schedule measurements every 500 ms
  scheduler.begin(500, processMeasurement, nullptr);
  sensor.addMessageHandler(printer);
  sensor.begin();
}

void processMeasurement(void*) {
  // Set the sensor's transform to world frame
  auto tf_lidar2world = tf.getTransform(lidar, world);
  sensor.setTransform(tf_lidar2world);

  // Read distance in cm from the ultrasonic sensor
  int distance_cm = ultrasonic.read();

  // Convert to meters and update the RangeSensor
  sensor.setObstacleDistance(Distance(distance_cm, DistanceUnit::CM));
}

void loop() {
  scheduler.run();
}
