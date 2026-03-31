#include "TinyRobotics/planning/Path.h"
#pragma once
#include <math.h>

#include "TinyRobotics/control/MotionState3D.h"
#include "TinyRobotics/control/PIDController.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
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
    pidX.begin(0.1f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f);
    pidY.begin(0.1f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f);
    pidZ.begin(0.1f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f);
    pidYaw.begin(0.1f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f);
    pidPitch.begin(0.1f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f);
    pidRoll.begin(0.1f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f);
  }

  void configurePositionPID(float dt, float maxOut, float minOut, float kp,
                            float ki, float kd) {
    pidX.begin(dt, maxOut, minOut, kp, ki, kd);
    pidY.begin(dt, maxOut, minOut, kp, ki, kd);
    pidZ.begin(dt, maxOut, minOut, kp, ki, kd);
  }
  void configureOrientationPID(float dt, float maxOut, float minOut, float kp,
                               float ki, float kd) {
    pidYaw.begin(dt, maxOut, minOut, kp, ki, kd);
    pidPitch.begin(dt, maxOut, minOut, kp, ki, kd);
    pidRoll.begin(dt, maxOut, minOut, kp, ki, kd);
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
    if (!path.isEmpty()) {
      target = MotionState3D(
          path[0], target.getOrientation(), Speed3D(0, 0, 0, SpeedUnit::MPS),
          AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSecond));
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
    float vyaw = pidYaw.calculate(wrapAngle(target.getOrientation().yaw),
                                  wrapAngle(motionState.getOrientation().yaw));
    float vpitch =
        pidPitch.calculate(wrapAngle(target.getOrientation().pitch),
                           wrapAngle(motionState.getOrientation().pitch));
    float vroll =
        pidRoll.calculate(wrapAngle(target.getOrientation().roll),
                          wrapAngle(motionState.getOrientation().roll));

    linearCmd = Speed3D(vx, vy, vz, SpeedUnit::MPS);
    angularCmd = AngularVelocity3D(vyaw, vpitch, vroll,
                                   AngularVelocityUnit::RadPerSecond);

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
      AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSecond);
  bool (*onGoalCallback)(void*) = nullptr;
  void* onGoaldRef = this;

  bool advanceWaypoint(float positionTolerance = 0.1f) {
    if (path.isEmpty()) return false;
    auto tgt = path[0];
    float dx = tgt.x - motionState.getPosition().x;
    float dy = tgt.y - motionState.getPosition().y;
    float dz = tgt.z - motionState.getPosition().z;
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);
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
              AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSecond));
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
        angularCmd =
            AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSecond);
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
          if (circlePhase > 2 * M_PI) circlePhase -= 2 * M_PI;
        }
        // Compute new target on the circle (XY plane, keep Z constant)
        float x = circleCenter.x + circleRadius * cosf(circlePhase);
        float y = circleCenter.y + circleRadius * sinf(circlePhase);
        float z = circleCenter.z;
        // Optionally, face tangent to the circle (yaw)
        float yaw = atan2f(y - circleCenter.y, x - circleCenter.x) + M_PI_2;
        Orientation3D orientation(yaw, target.getOrientation().pitch,
                                  target.getOrientation().roll);
        target = MotionState3D(
            Coordinate<DistanceM>(x, y, z), orientation,
            Speed3D(0, 0, 0, SpeedUnit::MPS),
            AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSecond));
        break;
    }
  }

  static float wrapAngle(float a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }
};

}  // namespace tinyrobotics
