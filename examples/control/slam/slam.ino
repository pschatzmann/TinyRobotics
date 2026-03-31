// TinyRobotics SLAM2D Example with LDS Lidar, CarAckerman, and
// MotionController2D
#include <LDS.h>  // Assumes LDS library is installed and provides LDS lidar interface
#include <TinyRobotics.h>
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
