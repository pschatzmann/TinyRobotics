/*
 * TinyRobotics SLAM2D Example
 *
 * Demonstrates Simultaneous Localization and Mapping (SLAM) using:
 *   - LDS Lidar sensor (requires LDS library)
 *   - Adafruit ICM20948 IMU
 *   - CarAckerman vehicle model
 *   - MotionController2D for path following
 *
 * Features:
 *   - Real-time SLAM with 2D Lidar and IMU fusion
 *   - Path planning using A* algorithm
 *   - Waypoint navigation and vehicle control
 *
 * Hardware:
 *   - Compatible with Arduino boards
 *   - Connect LDS Lidar and ICM20948 IMU as per their documentation
 *   - CarAckerman: specify motor/steering pins as needed
 *
 * Usage:
 *   - Upload to your Arduino-compatible board
 *   - Monitor Serial output for pose and navigation info
 *   - Adjust waypoints and parameters as needed
 * 
 * Dependency:
 * - https://github.com/kaiaai/LDS
 * - Adafruit_ICM20X
 */

#include <LDS.h>  // https://github.com/kaiaai/LDS
#include <Adafruit_ICM20948.h>

// SLAM2D instance
Adafruit_ICM20948 icm;
LDS lidar;
IMU2D<float> imu;
CarAckerman car(5, 6, 9, 10);
MotionController2D<float> controller(imu, car);
Frame2D world(FrameType::WORLD, 0);
Frame2D base(FrameType::BASE, 0, world, Transform2D(0, 0, 0));
Frame2D lidarFrame(FrameType::LIDAR, 0, base, Transform2D(0.15, 0, 0)); // Lidar 15cm ahead of base
// SLAM2D(dx, dy, resolution, world, base, lidar, imu2d)
SLAM2D<float> slam(25.0, 25.0, 0.10, world, base, lidarFrame, imu2d);
AStar<GridMap<CellState>, Coordinate<float>> astar;
Path plannedPath;
Scheduler imuScheduler;
Scheduler slamScheduler;

void setup() {
  Serial.begin(115200);
  // Initialize Lidar
  if (!lidar.begin()) {
    Serial.println("Lidar initialization failed!");
    while (1);
  }

  // Initialize ICM20948 IMU
  if (!icm.begin_I2C()) {
    Serial.println("ICM20948 not found");
    while (1);
  }
  Serial.println("ICM20948 found!");


  // Initialize car and controller
  car.begin();
  controller.begin();
  imuScheduler.schedule(updateIMU, 10);   // IMU update every 10ms
  slamScheduler.schedule(updateSLAM, 100); // SLAM update every 100ms
}


void updateIMU() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float gz = gyro.gyro.z;
  imu.update(ax, ay, gz, millis());
  imu.publish();
}

void updateSLAM() {
  LDSScan scan = lidar.getScan();
  for (int i = 0; i < scan.count; ++i) {
    float angle = scan.angles[i];        // in radians
    float distance = scan.distances[i];  // in meters
    auto local = Coordinate<float>(distance * cos(angle), distance * sin(angle), 0);
    auto worldPt = frameMgr.toWorld(lidarFrame, local);
    slam.addLidarPointWorld(worldPt.x, worldPt.y);
  }
  slam.update();

  auto pose = slam.getPose();
  controller.setPose(pose.x, pose.y, pose.theta);
  controller.update();

  Serial.print("X: "); Serial.print(pose.x);
  Serial.print(" Y: "); Serial.print(pose.y);
  Serial.print(" Theta: "); Serial.print(pose.theta);
  Serial.print(" | Target: ");
  Serial.print(waypoints[currentWaypoint].x); Serial.print(", ");
  Serial.println(waypoints[currentWaypoint].y);

  if (controller.isAtTarget()) {
    currentWaypoint = (currentWaypoint + 1) % numWaypoints;
    controller.setWaypoints(&waypoints[currentWaypoint], 1);
  }
}

void loop() {
  imuScheduler.run();
  slamScheduler.run();
}
