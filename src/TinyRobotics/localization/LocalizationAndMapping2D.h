#pragma once
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/imu/IMU2D.h"
#include "TinyRobotics/maps/GridBitMap.h"
#include "TinyRobotics/sensors/RangeSensor.h"
#include "TinyRobotics/planning/IFrontierExplorer.h"

namespace tinyrobotics {
/**
 * @class LocalizationAndMapping2D
 * @ingroup slam
 * @brief Modular 2D Simultaneous Localization and Mapping (SLAM) system with
 * flexible map, IMU, and exploration integration.
 *
 * This class implements a highly modular and extensible 2D SLAM system for
 * occupancy grid mapping, pose estimation, sensor fusion, and autonomous
 * exploration.
 *
 * ## Features
 * - **Template Parameters:**
 *   - `T`: Scalar type for coordinates and math (default: DistanceM)
 * IFrontierExplorer<T><T>)
 * - **Occupancy Grid Mapping:** Maintains a 2D grid map for the environment,
 * supporting any compatible map type via IMap<T>.
 * - **Pose Estimation:** Integrates IMU/odometry for robot pose tracking and
 * frame transforms (IMotionState2D).
 * - **Sensor Fusion:** Supports range sensors (e.g., Lidar) for obstacle
 * detection and map updates.
 * - **Frontier-Based Exploration:** Integrates a flexible frontier explorer for
 * autonomous navigation.
 * - **Frame Management:** Uses Frame2D and FrameMgr2D for coordinate transforms
 * between world, base, and sensor frames.
 * - **Flexible Integration:** All major components (map, IMU, explorer) are
 * passed by reference for easy swapping and testing.
 * - **Message Bus:** Inherits from MessageSource for integration with
 * message-based architectures.
 *
 * ## Template Parameters
 * @tparam T                  Scalar type for coordinates and math (default:
 * DistanceM)
 * @tparam IFrontierExplorer<T>  The frontier exploration strategy (default:
 * IFrontierExplorer<T><T>)
 *
 * ## Ownership & Lifetime
 * - All major components (map, IMU, explorer) are passed by reference and must
 * outlive the LocalizationAndMapping2D instance.
 *
 * ## Usage Example
 * @code
 *   GridBitMap map;
 *   IMU2D<float> imu;
 *   Frame2D world, base, lidar;
 *   FrontierExplorer<T> explorer(map);
 *   LocalizationAndMapping2D slam(map, explorer, world, base, lidar, imu);
 *   slam.begin();
 *   // ...
 * @endcode
 */

template <typename T>
class LocalizationAndMapping2D : public MessageSource {
 public:
  LocalizationAndMapping2D(IMap<T>& map, IFrontierExplorer<T>& explorer,
                           Frame2D& world, Frame2D& base, Frame2D& lidar,
                           IMotionState2D& imu)
      : explorer_(explorer),
        map_(map),
        world_(world),
        base_(base),
        lidar_(lidar),
        imu2d_(imu) {
    callbackHandler_.setValueCallback(onIMUPublished, this);
    imu2d_.subscribe(callbackHandler_);
  }

  /// Initialize SLAM system (IMU and range sensor)
  bool begin() {
    // update IMU
    Transform2D& transform = base_.getTransform();
    // Initialize IMU with the initial pose of the base frame
    bool rcIMU = imu2d_.begin(transform);
    bool rcSensor = rangeSensor_.begin();
    return rcIMU && rcSensor;
  }

  void end() {
    imu2d_.end();
    rangeSensor_.end();
  }

  /// Register Range Measurement from Lidar
  void addRangeMeasurement(Distance dist, Angle angle, CellState state) {
    addRangeMeasurement(dist.getValue(DistanceUnit::M),
                        angle.getValue(AngleUnit::DEG), state);
  }

  /// Register Range Measurement from Lidar
  void addRangeMeasurement(float distanceM, float angleDeg, CellState state) {
    rangeSensor_.setTransform(
        tf_.getTransform(lidar_, world_));  // Ensure transform is set
    rangeSensor_.setObstacle(distanceM, angleDeg);
    Coordinate obs;
    if (rangeSensor_.getObstacleCoordinate(obs)) {
      // Transform obstacle coordinate from lidar to world frame
      Transform2D tf_lidar2world = tf_.getTransform(lidar_, world_);
      Coordinate<float> obs_world = tf_lidar2world.apply(obs);
      // Get sensor (lidar) position in world frame
      Coordinate<float> sensor_world =
          tf_lidar2world.apply(Coordinate<float>(0, 0));
      // Mark free cells between sensor and obstacle, and mark obstacle as
      // OCCUPIED
      markRay(sensor_world, obs_world, state);
    }
  }

  /**
   * @brief Find the next frontier cell (boundary between known free and
   * unknown space).
   * @return World coordinates of the next frontier cell, or (0,0) if none
   * found.
   */
  bool getNextFrontier(Coordinate<float>& result) const {
    return explorer_.getNextFrontier(result);
  }

  /// Subscribe to imu and range sensor messages
  void subscribe(MessageHandler& handler) {
    imu2d_.subscribe(handler);
    rangeSensor_.subscribe(handler);
  }

  void unsubscribeAll() {
    imu2d_.unsubscribeAll();
    rangeSensor_.unsubscribeAll();
  }

  /// Access to map
  IMap<T>& getMap() { return map_; }

  /// Access to IMU state
  IMotionState2D& getIMU() { return imu2d_; }

  IFrontierExplorer<T>& getFrontierExplorer() { return explorer_; }

 protected:
  IFrontierExplorer<T>& explorer_;
  RangeSensor<T> rangeSensor_;
  IMotionState2D& imu2d_;
  IMap<T>& map_;
  CallbackMessageHandler callbackHandler_;
  // Frame management
  Frame2D& world_;
  Frame2D& base_;
  Frame2D lidar_;
  FrameMgr2D tf_;

  /**
   * @brief Mark all cells between the sensor and the obstacle as FREE, and
   * the obstacle cell as OCCUPIED.
   * @param sensor_world World coordinates of the sensor
   * @param obs_world World coordinates of the obstacle
   */
  void markRay(const Coordinate<float>& sensor,
               const Coordinate<float>& obstacle, CellState state) {
    auto points = sensor.interpolateTo(obstacle, map_.getResolution());
    for (auto& loc : points) {
      if (map_.isValid(loc)) map_.setCell(loc, CellState::FREE);
    }
    if (map_.isValid(obstacle)) {
      map_.setCell(obstacle, state);
    }
  }

  static bool onIMUPublished(const Message<float>& msg, void* ref) {
    // Update the base position
    LocalizationAndMapping2D& self = *(LocalizationAndMapping2D*)ref;
    self.base_.setTransform(self.imu2d_.getTransform());
    return true;
  }
};

}  // namespace tinyrobotics