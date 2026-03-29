#pragma once
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/imu/IMU2D.h"
#include "TinyRobotics/maps/GridMap.h"
#include "TinyRobotics/sensors/RangeSensor.h"

namespace tinyrobotics {

/**
 * @class SLAM2D
 * @brief 2D Simultaneous Localization and Mapping (SLAM) with frame transforms
 * and message integration.
 *
 * This class implements a modular 2D SLAM system for occupancy grid mapping,
 * pose estimation, and sensor fusion. It integrates IMU and range sensor data,
 * uses a frame hierarchy (world, base, lidar) for coordinate transforms, and
 * supports message-based communication via MessageSource inheritance.
 *
 * Features:
 * - Inherits from MessageSource for message bus integration
 * - Maintains an occupancy grid map (GridMap<CellState>)
 * - Integrates IMU2D for pose estimation and publishes IMU state
 * - Integrates RangeSensor for obstacle detection
 * - Uses Frame2D and FrameMgr2D for coordinate transforms between frames
 * - Efficiently updates the map: marks all cells between sensor and obstacle as
 * FREE, and the obstacle cell as OCCUPIED
 * - Provides exploration support via getNextFrontier(result) (frontier-based
 * exploration, returns true if a frontier is found and writes coordinates to
 * result)
 * - Supports message subscription/unsubscription for IMU and range sensor
 *
 * Typical usage:
 * @code
 *   Frame2D world, base, lidar;
 *   FrameMgr2D tf;
 *   IMU2D<float> imu;
 *   SLAM2D slam(dx, dy, resolution, world, base, lidar, imu); // Pass IMU2D as parameter
 *   slam.begin();
 *   // Update IMU values (not via SLAM2D, but via the IMU2D instance)
 *   imu.update(accelX, accelY, gyroZ, nowMillis); // IMU update
 *   imu.publish(); // Publish IMU state and update base frame
 *   slam.addRangeMeasurement(distance, angle);    // Lidar update
 *   auto& map = slam.getMap();
 *   Coordinate<float> frontier;
 *   if (slam.getNextFrontier(frontier)) {
 *     // Use frontier coordinate
 *   }
 * @endcode
 */

template <typename T = float>
class SLAM2D : public MessageSource {
 public:
  SLAM2D(T dx, T dy, T resolution, const Frame2D& world, const Frame2D& base,
         const Frame2D& lidar, IMU2D<T>& imu)
      : map_(static_cast<int>(dx / resolution),
             static_cast<int>(dy / resolution), resolution),
        world_(world),
        base_(base),
        lidar_(lidar) {
    // update position when publish is called on imu
    imu.setOnPublishedCallback(
        [this](void* ref) {
          SLAM2D& self = *(SLAM2D*)ref;
          // Update the base position
          Transform2D t(self.imu2d_.getPosition().x,
                        self.imu2d_.getPosition().y,
                        self.imu2d_.getHeadingAngle(AngleUnit::DEG));
          self.base_.setTransform(t);
        },
        this);
  }

  /// Initialize SLAM system (IMU and range sensor)
  bool begin() {
    // update IMU
    Transform2D& transform = base_.getTransform();
    // Initialize IMU with the initial pose of the base frame
    bool rcIMU = imu2d_.begin(transform.heading_deg, transform.pos);
    bool rcSensor = rangeSensor_.begin();
    return rcIMU && rcSensor;
  }

  void end() {
    imu2d_.end();
    rangeSensor_.end();
  }

  /// Register Range Measurement from Lidar
  void addRangeMeasurement(Distance dist, Angle angle) {
    addRangeMeasurement(dist.getDistance(DistanceUnit::M),
                        angle.getAngle(AngleUnit::DEG));
  }
  /// Register Range Measurement from Lidar
  void addRangeMeasurement(T distanceM, T angleDeg) {
    rangeSensor_.setTransform(
        tf_.getTransform(lidar_, world_));  // Ensure transform is set
    rangeSensor_.setObstacle(distanceM, angleDeg);
    Coordinate<T> obs;
    if (rangeSensor_.getObstacleCoordinate(obs)) {
      // Transform obstacle coordinate from lidar to world frame
      Transform2D tf_lidar2world = tf_.getTransform(lidar_, world_);
      Coordinate<T> obs_world = tf_lidar2world.apply(obs);
      // Get sensor (lidar) position in world frame
      Coordinate<T> sensor_world = tf_lidar2world.apply(Coordinate<T>(0, 0));
      // Mark free cells between sensor and obstacle, and mark obstacle as
      // OCCUPIED
      markFreeCellsBetweenSensorAndObstacle(sensor_world, obs_world);
    }
  }
  /**
   * @brief Find the next frontier cell (boundary between known free and
   * unknown space).
   * @return World coordinates of the next frontier cell, or (0,0) if none
   * found.
   */
  bool getNextFrontier(Coordinate<T>& result) const {
    int xCount = map_.getXCount();
    int yCount = map_.getYCount();
    // For each cell, check if it is free and adjacent to unknown
    for (int cy = 0; cy < yCount; ++cy) {
      for (int cx = 0; cx < xCount; ++cx) {
        CellState state;
        if (!map_.getCell(cx, cy, state)) continue;
        if (state != CellState::FREE) continue;
        // Check 4-neighborhood for unknown
        bool isFrontier = false;
        for (int dx = -1; dx <= 1 && !isFrontier; ++dx) {
          for (int dy = -1; dy <= 1 && !isFrontier; ++dy) {
            if ((dx == 0 && dy == 0) || abs(dx) + abs(dy) != 1)
              continue;  // 4-neighborhood only
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx < 0 || nx >= xCount || ny < 0 || ny >= yCount) continue;
            CellState neighborState;
            if (!map_.getCell(nx, ny, neighborState)) continue;
            if (neighborState == CellState::UNKNOWN) {
              isFrontier = true;
            }
          }
        }
        if (isFrontier) {
          // Convert cell to world coordinates and return
          result = map_.toWorld(cx, cy);
          return true;
        }
      }
    }
    // No frontier found
    result = Coordinate<T>(0, 0);
    return false;
  }

  /// Access to map
  const GridMap<CellState>& getMap() const { return map_; }

  /// Access to IMU state
  const IMU2D<T>& getIMU() const { return imu2d_; }

  /// Subscribe to imu and range sensor messages
  void subscribe(MessageHandler& handler) {
    imu2d_.subscribe(handler);
    rangeSensor_.subscribe(handler);
  }

  void unsubscribeAll() {
    imu2d_.unsubscribeAll();
    rangeSensor_.unsubscribeAll();
  }

 protected:
  RangeSensor<T> rangeSensor_;
  IMU2D<T>& imu2d_;
  GridMap<CellState> map_;
  // Frame management
  Frame2D world_;
  Frame2D base_;
  Frame2D lidar_;
  FrameMgr2D tf_;

  /**
   * @brief Mark all cells between the sensor and the obstacle as FREE, and
   * the obstacle cell as OCCUPIED.
   * @param sensor_world World coordinates of the sensor
   * @param obs_world World coordinates of the obstacle
   */
  void markFreeCellsBetweenSensorAndObstacle(const Coordinate<T>& sensor_world,
                                             const Coordinate<T>& obs_world) {
    GridMap<CellState>::Cell cell_obs, cell_sensor;
    if (map_.worldToCell(obs_world.x, obs_world.y, cell_obs) &&
        map_.worldToCell(sensor_world.x, sensor_world.y, cell_sensor)) {
      // Bresenham's line algorithm to mark free cells
      int x0 = static_cast<int>(cell_sensor.cx);
      int y0 = static_cast<int>(cell_sensor.cy);
      int x1 = static_cast<int>(cell_obs.cx);
      int y1 = static_cast<int>(cell_obs.cy);
      int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
      int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
      int err = dx + dy, e2;
      int x = x0, y = y0;
      while (true) {
        if (x == x1 && y == y1) break;  // Stop before marking obstacle cell
        map_.setCell(x, y, CellState::FREE);
        e2 = 2 * err;
        if (e2 >= dy) {
          err += dy;
          x += sx;
        }
        if (e2 <= dx) {
          err += dx;
          y += sy;
        }
      }
      // Mark the obstacle cell as OCCUPIED
      map_.setCell(x1, y1, CellState::OCCUPIED);
    }
  }
};

}  // namespace tinyrobotics