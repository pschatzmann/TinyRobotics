#pragma once
#include <cmath>
#include "Arduino.h" // for millis
#include "TinyRobotics/control/MotionState3D.h"
#include "TinyRobotics/control/PIDController.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/planning/Path.h"
#include "TinyRobotics/units/AngularVelocity.h"
#include "TinyRobotics/units/Speed.h"

namespace tinyrobotics {

/// Action to take when reaching the goal (last waypoint)
enum class OnGoalAction { Stop, HoldPosition, Circle };

/**
 * @class MotionController3D
 * @ingroup control
 * @brief 3D motion/path controller using PID for position and orientation.
 *
 * This class provides high-level 3D path following and pose control for robots,
 * drones, or vehicles. It uses independent PID controllers for each axis (x, y,
 * z) and orientation (yaw, pitch, roll).
 *
 * ## Features
 * - Accepts a reference to an IMotionState3D for real-time feedback
 * - Supports a path of 3D waypoints (Path<Coordinate<DistanceM>>)
 * - Automatically advances to the next waypoint when the current one is reached
 * - PID gains and limits are configurable for both position and orientation
 * - Provides update() to compute new commands, and getters for linear/angular
 * velocity commands
 * - begin() initializes the target from the first path coordinate
 *
 * ## Usage
 * 1. Create with a reference to your IMotionState3D implementation (e.g.,
 * IMU3D)
 * 2. Set a path of waypoints with setPath()
 * 3. Call begin() to initialize the first target
 * 4. In your control loop:
 *    - Call update() to compute new commands
 *    - Use getLinearCommand() and getAngularCommand() for actuation
 *
 * ## Limitations
 * - Only the position is updated from the path; orientation, speed, and angular
 * velocity are kept from the previous target
 *
 * ## Example
 * @code
 * MotionController3D controller(imu3d);
 * controller.setPath(path);
 * controller.begin();
 * while (!controller.isPathComplete()) {
 *   controller.update();
 *   auto v = controller.getLinearCommand();
 *   auto w = controller.getAngularCommand();
 *   // send v, w to actuators
 *   controller.advanceWaypoint();
 * }
 * @endcode
 */
class MotionController3D {
 public:
  MotionController3D(IMotionState3D& motionStateRef, OnGoalAction onGloal,
                     float positionToleranceM = 2.0)
      : motionState(motionStateRef),
        positionToleranceM(positionToleranceM),
        onGoalAction(onGloal) {
    // Default PID: dt=0.1s, max=1.0, min=-1.0, kp=1.0, ki=0.0, kd=0.0
    pidX.begin(0.1f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    pidY.begin(0.1f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    pidZ.begin(0.1f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    pidYaw.begin(0.1f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    pidPitch.begin(0.1f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    pidRoll.begin(0.1f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
  }

  void configurePositionPID(float dt, float maxOut, float minOut, float kp,
                            float ki, float kd) {
    pidX.begin(dt, minOut, maxOut, kp, ki, kd);
    pidY.begin(dt, minOut, maxOut, kp, ki, kd);
    pidZ.begin(dt, minOut, maxOut, kp, ki, kd);
  }
  void configureOrientationPID(float dt, float minOut, float maxOut, float kp,
                               float ki, float kd) {
    pidYaw.begin(dt, minOut, maxOut, kp, ki, kd);
    pidPitch.begin(dt, minOut, maxOut, kp, ki, kd);
    pidRoll.begin(dt, minOut, maxOut, kp, ki, kd);
  }

  void setPath(Path<Coordinate<DistanceM>> path) {
    this->path = path;
    circleInitialized = false;
  }

  /**
   * @brief Initialize controller and set target from first path coordinate if
   * available.
   */
  void begin() {
    updateCount = 0;
    updateStartTimeMs = 0;
    updateEndTimeMs = 0;
    dtSetFromUpdates = false;

    if (!path.isEmpty()) {
      target = MotionState3D(
          path[0], target.getOrientation(), Speed3D(0, 0, 0, SpeedUnit::MPS),
          AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSec));
      is_active = true;
    }
  }

  /// Stop the controller and the vehicle
  void end() { is_active = false; }

  /**
   * @brief Update the controller and compute new commands.
   * @return true if we still have a path to follow, false if path is complete
  or controller is inactive.
   */
  bool update() {
    if (!is_active) return false;
    if (!initializeDtFromUpdates()) return true;
    advanceWaypoint(positionToleranceM);
    if (path.isEmpty()) {
      return false;
    }
    // Use the referenced motionState for current state
    float vx =
        pidX.calculate(target.getPosition().x, motionState.getPosition().x);
    float vy =
        pidY.calculate(target.getPosition().y, motionState.getPosition().y);
    float vz =
        pidZ.calculate(target.getPosition().z, motionState.getPosition().z);
    float vyaw =
        pidYaw.calculate(normalizeAngleRad(target.getOrientation().yaw),
                         normalizeAngleRad(motionState.getOrientation().yaw));
    float vpitch = pidPitch.calculate(
        normalizeAngleRad(target.getOrientation().pitch),
        normalizeAngleRad(motionState.getOrientation().pitch));
    float vroll =
        pidRoll.calculate(normalizeAngleRad(target.getOrientation().roll),
                          normalizeAngleRad(motionState.getOrientation().roll));

    linearCmd = Speed3D(vx, vy, vz, SpeedUnit::MPS);
    angularCmd =
        AngularVelocity3D(vyaw, vpitch, vroll, AngularVelocityUnit::RadPerSec);

    return true;
  }

  Speed3D getLinearCommand() const { return linearCmd; }

  AngularVelocity3D getAngularCommand() const { return angularCmd; }

  MotionState3D getTarget() const { return target; }

  bool isActive() const { return is_active; }

  /**
   * @brief Set the OnGoalAction behavior (Stop, HoldPosition, Circle).
   * Resets circle mode state if changed.
   */
  void setOnGoalAction(OnGoalAction action) {
    onGoalAction = action;
    circleInitialized = false;
  }

  /**
   * @brief Set a custom callback to be called when reaching the goal. The
   * callback should return true if it handled the goal action, or false to
   * allow default handling.
   */
  void setOnGoalCallback(bool (*callback)(void*), void* ref = nullptr) {
    onGoalCallback = callback;
    if (ref != nullptr) onGoaldRef = ref;
  }

  /**
   * @brief Set the radius for circular motion (meters).
   */
  void setCircleRadius(float radius) {
    circleRadius = radius;
    circleInitialized = false;
  }

  /**
   * @brief Set the angular speed for circular motion (radians per update).
   */
  void setCircleAngularSpeed(float angularSpeed) {
    circleAngularSpeed = angularSpeed;
  }

 protected:
  bool is_active = false;
  float positionToleranceM;
  OnGoalAction onGoalAction;
  Path<Coordinate<DistanceM>> path;
  IMotionState3D& motionState;
  MotionState3D target =
      MotionState3D(Coordinate<DistanceM>(0, 0, 0), Orientation3D(), Speed3D(),
                    AngularVelocity3D());
  PIDController<float> pidX, pidY, pidZ;
  PIDController<float> pidYaw, pidPitch, pidRoll;
  Speed3D linearCmd = Speed3D(0, 0, 0, SpeedUnit::MPS);
  AngularVelocity3D angularCmd =
      AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSec);
  bool (*onGoalCallback)(void*) = nullptr;
  void* onGoaldRef = this;
  // For dynamic dt measurement (simplified)
  int updateCount = 0;
  unsigned long updateStartTimeMs = 0;
  unsigned long updateEndTimeMs = 0;
  bool dtSetFromUpdates = false;

  /// Handles dt initialization from first 10 updates, but does not block
  /// control logic
  bool initializeDtFromUpdates() {
    unsigned long nowMs = millis();
    if (dtSetFromUpdates) return true;
    if (updateCount == 0) {
      updateStartTimeMs = nowMs;
    }
    updateCount++;
    if (updateCount == 11) {
      updateEndTimeMs = nowMs;
      float avgMs = (updateEndTimeMs - updateStartTimeMs) / 10.0f;
      float dt = avgMs / 1000.0f;  // convert ms to seconds
      // Reconfigure all PIDs with new dt, keep other params
      pidX.setDt(dt);
      pidY.setDt(dt);
      pidZ.setDt(dt);
      pidYaw.setDt(dt);
      pidPitch.setDt(dt);
      pidRoll.setDt(dt);
      dtSetFromUpdates = true;
    }
    return dtSetFromUpdates;
  }

  bool advanceWaypoint(float positionTolerance = 0.1f) {
    if (path.isEmpty()) return false;
    auto tgt = path[0];
    float dx = tgt.x - motionState.getPosition().x;
    float dy = tgt.y - motionState.getPosition().y;
    float dz = tgt.z - motionState.getPosition().z;
    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < positionTolerance) {
      path.removeHead();
      if (!path.isEmpty()) {
        // Set next waypoint as new target (keep orientation/vel from previous
        // target)
        if (path.size() == 1) {
          // hover at coordinate
          target = MotionState3D(
              path[0], target.getOrientation(),
              Speed3D(0, 0, 0, SpeedUnit::MPS),
              AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSec));
        } else {
          target =
              MotionState3D(path[0], target.getOrientation(), target.getSpeed(),
                            target.getAngularVelocity());
        }
      } else {
        // Path is now empty, trigger on-goal action
        handleOnGoalAction();
      }
      return true;
    }
    return false;
  }
  // For Circle mode
  float circlePhase = 0.0f;
  float circleRadius = 5.0f;        // meters
  float circleAngularSpeed = 0.5f;  // radians per update
  Coordinate<DistanceM> circleCenter;
  bool circleInitialized = false;

  void handleOnGoalAction() {
    // process callback
    if (onGoalCallback) {
      if (onGoalCallback(onGoaldRef)) {
        // Callback handled the goal action, so we can return early
        return;
      }
    }
    switch (onGoalAction) {
      case OnGoalAction::Stop:
        // Set all commands to zero and deactivate controller
        linearCmd = Speed3D(0, 0, 0, SpeedUnit::MPS);
        angularCmd = AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSec);
        is_active = false;
        break;
      case OnGoalAction::HoldPosition:
        // Keep controller active, hold at last target (do nothing)
        break;
      case OnGoalAction::Circle:
        // Move in a circle around the last goal
        if (!circleInitialized) {
          circleCenter = motionState.getPosition();
          circlePhase = 0.0f;
          circleInitialized = true;
        } else {
          circlePhase += circleAngularSpeed;
          if (circlePhase > 2.0f * static_cast<float>(M_PI))
            circlePhase -= 2.0f * static_cast<float>(M_PI);
        }
        // Compute new target on the circle (XY plane, keep Z constant)
        float x = circleCenter.x + circleRadius * std::cos(circlePhase);
        float y = circleCenter.y + circleRadius * std::sin(circlePhase);
        float z = circleCenter.z;
        // Optionally, face tangent to the circle (yaw)
        float yaw = std::atan2(y - circleCenter.y, x - circleCenter.x) +
                    static_cast<float>(M_PI_2);
        Orientation3D orientation(yaw, target.getOrientation().pitch,
                                  target.getOrientation().roll);
        target = MotionState3D(
            Coordinate<DistanceM>(x, y, z), orientation,
            Speed3D(0, 0, 0, SpeedUnit::MPS),
            AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSec));
        break;
    }
  }

  // Removed: use normalizeAngleRad directly
};

}  // namespace tinyrobotics
