#pragma once
#include "MotionState2D.h"
#include "PIDController.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/imu/IMU2D.h"
#include "TinyRobotics/planning/Path.h"

namespace tinyrobotics {

/**
 * @class MotionController2D
 * @ingroup control
 * @brief 2D motion controller for path following and vehicle control.
 *
 * This class implements a 2D motion controller for robotic vehicles using PID
 * control for both speed (throttle) and steering. It supports path following,
 * smooth acceleration/deceleration, and integrates with an IMU for feedback.
 *
 * - Uses a configurable PID controller for speed-to-throttle mapping.
 * - Uses a configurable PID controller for steering angle correction.
 * - Accepts a path (sequence of waypoints) and drives the vehicle to follow it.
 * - Automatically slows down as the vehicle approaches the goal.
 * - Designed for modular use with different vehicle types and IMU sensors.
 * - Vehicle needs to suscribe to receive control messages (throttle and
 * steering angle).
 *
 * @tparam T Numeric type for calculations (default: float)
 *
 * @author Phil Schatzmann
 */
template <typename T = float>
class MotionController2D : public MessageSource {
 public:
  MotionController2D(IMotionState2D& motionState, float maxSpeedKmh = 10,
                     float accelDistanceM = 2.0f)
      : motionStateSource(motionState),
        accelDistanceM(accelDistanceM),
        maxSpeedKmh(maxSpeedKmh) {
    configureSteeringPID(0.1f, 30.0f, -30.0f, 1.0f, 0.0f,
                         0.1f);  // default steering PID
  configureSpeedPID(
    0.1f, 100.0f, 0.0f, 2.0f, 0.2f,
    0.5f);  // speed PID: reduced gains for stability
  }

  MotionController2D(IMotionState2D& motionState, Speed maxSpeedKmh,
                     Distance accelDistanceM)
      : MotionController2D(motionState, maxSpeedKmh.getValue(SpeedUnit::KPH),
                           accelDistanceM.getValue(DistanceUnit::M)) {}

  /**
   * @brief Configure the PID controller for speed-to-throttle mapping.
   * @param dt Time step (seconds)
   * @param maxOut Maximum output (throttle)
   * @param minOut Minimum output (throttle)
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void configureSpeedPID(float dt, float maxOut, float minOut, float kp,
                         float ki, float kd) {
    pidSpeed_.begin(dt, maxOut, minOut, kp, ki, kd);
  }

  /**
   * @brief Configure the PID controller for steering.
   * @param dt Time step (seconds)
   * @param maxOut Maximum output
   * @param minOut Minimum output
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void configureSteeringPID(float dt, float maxOut, float minOut, float kp,
                            float ki, float kd) {
    pidSteering_.begin(dt, maxOut, minOut, kp, ki, kd);
  }

  /// Defines the path to follow as a sequence of waypoints
  void setPath(Path<Coordinate<DistanceM>> path) { this->path = path; }

  /// Adds a single waypoint to the path (appended to the end)
  void addWaypoint(Coordinate<DistanceM> target) {
    this->path.addWaypoint(target);
  }

  /**
   * @brief Set the target accuracy (meters) for reaching waypoints.
   * @param accuracyM The new target accuracy in meters.
   */
  void setTargetAccuracy(float accuracyM) { targetAccuracyM = accuracyM; }

  /// Start the controller and initialize state
  bool begin() {
    is_active = true;
    has_distance = false;
    if (!hasStartCoordinate) {
      startCoordinate = motionStateSource.getPosition();
      hasStartCoordinate = true;
    }
    return true;
  }

  /// Stop the controller and the vehicle
  void end() { is_active = false; }

  /// Main control loop to be called periodically (e.g., in a timer or main
  /// loop)
  void update() {
    if (!is_active) return;
    Coordinate<DistanceM> currentPos = motionStateSource.getPosition();
    Coordinate<DistanceM> targetPos = path[0];
    float currentHeading =
        motionStateSource.getHeading().getValue(AngleUnit::DEG);
    float desiredHeading = 0;
    if (!handleWaypoint(currentPos, targetPos, distanceToTargetM,
                        desiredHeading)) {
      // new waypoint
      return;
    }
    float headingError = computeHeadingError(desiredHeading, currentHeading);
    float currentSpeedKmh =
        motionStateSource.getSpeed().getValue(SpeedUnit::KPH);
    float distanceFromStartM =
        startCoordinate.distance(currentPos, DistanceUnit::M);
    updateSpeed(distanceFromStartM, distanceToTargetM, currentSpeedKmh);

    has_distance = true;
    sendControlMessages(distanceToTargetM, headingError, currentSpeedKmh);
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

  float getThrottlePercent() const { return resultThrottlePercent; }
  float getSteeringAngleDeg() const { return resultStreeringAngleDeg; }
  Angle getSteeringAngle() const {
    return Angle(resultStreeringAngleDeg, AngleUnit::DEG);
  }

  float getTargetAccuracy(DistanceUnit unit) const {
    return Distance(targetAccuracyM, DistanceUnit::M).getValue(unit);
  }

  /// Returns true if the goal has been reached
  bool isGoalReached() const {
    if (!has_distance) return false;
    return distanceToTargetM < targetAccuracyM;
  }

 protected:
  IMotionState2D& motionStateSource;
  Path<Coordinate<DistanceM>> path;
  bool is_active = false;
  bool has_distance = false;
  float accelDistanceM = 2.0f;
  float maxSpeedKmh = 10.0f;
  float desiredSpeedKmh = 0.0f;
  PIDController<float> pidSteering_;
  PIDController<float> pidSpeed_;
  Coordinate<DistanceM> startCoordinate;
  bool hasStartCoordinate = false;
  float targetAccuracyM = 0.10f;
  float distanceToTargetM = 0;
  bool (*onGoalCallback)(void*) = nullptr;
  void* onGoaldRef = this;
  float resultThrottlePercent = 0.0f;
  float resultStreeringAngleDeg = 0.0f;

  /// Calculate desired speed based on distance to target and distance from
  /// start
  void updateSpeed(float distanceFromStartM, float distanceToTarget,
                   float currentSpeedKmh) {
    // Compute a desired speed profile: slow down as you approach the target
    float minSpeedKmh = 0.0f;
    desiredSpeedKmh = maxSpeedKmh;  // * (distanceToTarget / accelDistanceM);
    if (distanceToTarget < accelDistanceM) {
      if (distanceToTarget < targetAccuracyM) {
        desiredSpeedKmh = 0.0f;  // Stop at the target
      } else {
        desiredSpeedKmh = maxSpeedKmh * (distanceToTarget / accelDistanceM);
      }
    }
    if (distanceFromStartM < accelDistanceM) {
      // avoid 0 start speed!
      if (distanceFromStartM == 0)
        distanceFromStartM = 0.1f;  // avoid division by zero
      desiredSpeedKmh = maxSpeedKmh * (distanceFromStartM / accelDistanceM);
    }
    if (desiredSpeedKmh > maxSpeedKmh) desiredSpeedKmh = maxSpeedKmh;
    if (desiredSpeedKmh < minSpeedKmh) desiredSpeedKmh = minSpeedKmh;
  }

  /// Handle the current waypoint: calculate distance and desired heading
  bool handleWaypoint(const Coordinate<DistanceM>& currentPos,
                      Coordinate<DistanceM>& targetPos,
                      float& distanceToTargetM, float& desiredHeadingDeg) {
    // Use class variable targetAccuracyM
    distanceToTargetM = 0.0f;
    while (distanceToTargetM < targetAccuracyM) {
      desiredHeadingDeg = currentPos.bearing(targetPos, AngleUnit::DEG);
      distanceToTargetM = currentPos.distance(targetPos, DistanceUnit::M);
      if (distanceToTargetM < targetAccuracyM) {
        path.removeHead();
        if (path.isEmpty()) {
          if (onGoalCallback) {
            onGoalCallback(onGoaldRef);
          }
          return false;
        }
        targetPos = path[0];
        return false;
      }
    }
    return true;
  }

  float computeHeadingError(float desiredHeading, float currentHeading) {
    float headingError = desiredHeading - currentHeading;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    return headingError;
  }

  void sendControlMessages(float distanceToTarget, float headingError,
                           float currentSpeedKmh) {
    Message<float> msg;
  float throttlePercent = pidSpeed_.calculate(desiredSpeedKmh, currentSpeedKmh);
  // Clamp throttle to [0, 100]
  if (throttlePercent > 100.0f) throttlePercent = 100.0f;
  if (throttlePercent < 0.0f && throttlePercent > -1.0f) throttlePercent = 0.0f;
  if (throttlePercent > 0.0f && throttlePercent < 1.0f) throttlePercent = 1.0f;  // avoid too low throttle
  if (throttlePercent < 0.0f) throttlePercent = 0.0f;
    msg.source = MessageOrigin::Autonomy;
    msg.content = MessageContent::Throttle;
    msg.unit = Unit::Percent;
    // Use PID to map speed to throttle
    msg.value = resultThrottlePercent = throttlePercent;
    sendMessage(msg);
    msg.content = MessageContent::SteeringAngle;
    msg.unit = Unit::AngleDegree;
    msg.value = resultStreeringAngleDeg =
        pidSteering_.calculate(0.0f, -headingError);
    sendMessage(msg);
    TRLogger.info(
        "MotionController2D: distanceToTarget=%.2f m, desiredSpeed=%.2f km/h, "
        "currentSpeed=%.2f km/h, throttle=%.1f%%, headingError=%.1f deg, "
        "steeringAngle=%.1f deg",
        distanceToTarget, desiredSpeedKmh, currentSpeedKmh,
        resultThrottlePercent, headingError, resultStreeringAngleDeg);
  }
};

}  // namespace tinyrobotics