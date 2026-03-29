/**
 * @file imu2d.ino
 * @brief Example: 2D Sensor Fusion with GY-912 ICM20948 and TinyRobotics IMU2D
 *
 * This example demonstrates how to use a GY-912 ICM20948 10DOF IMU with the TinyRobotics IMU2D class
 * for real-time 2D sensor fusion (position, velocity, heading) on Arduino/ESP32 platforms.
 *
 * - Reads accelerometer, gyroscope, and magnetometer data from the ICM20948 using the Adafruit_ICM20X library.
 * - Updates the IMU2D filter at 10ms intervals using the TinyRobotics Scheduler.
 * - Publishes fused state (heading, position, etc.) as JSON via MessageHandlerPrintJSON to Serial.
 *
 * Dependencies:
 *   - Adafruit_ICM20X (for ICM20948): https://github.com/adafruit/Adafruit_ICM20X
 *   - TinyRobotics (IMU2D, Scheduler, MessageHandlerPrintJSON)
 *
 * Output: JSON-formatted IMU2D state to Serial every 10ms.
 */

#include <Adafruit_ICM20948.h>
#include <TinyRobotics.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
IMU2D<float> imu2d;
MessageHandlerPrintJSON json(Serial);
Scheduler scheduler;

void updateIMU(void* ref) {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Update IMU2D with new sensor data
  imu2d.update(accel.acceleration.x, accel.acceleration.y, gyro.gyro.z,
               millis());
  // Update with magnetometer data for heading correction
  imu2d.updateMagnetometer(mag.magnetic.x, mag.magnetic.y);
  imu2d.publish();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) delay(10);
  }
  Serial.println("ICM20948 found!");

  // Subscribe to IMU2D messages for JSON output
  imu2d.subscribe(json);
  // initial position (0,0) and heading 0 radians north
  imu2d.begin(0.0f, Coordinate<float>(0, 0));
  scheduler.begin(10, updateIMU, nullptr);  // Schedule IMU update every 10 ms
}

void loop() { scheduler.run(); }