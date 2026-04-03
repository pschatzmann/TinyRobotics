// For dynamic dt measurement (simplified)
#pragma once
#include "MotionState2D.h"
#include "PIDController.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/imu/IMU2D.h"
#include "TinyRobotics/planning/Path.h"

namespace tinyrobotics {

/**
 * @brief Throttle control strategy selection.
 * @ingroup control
 */
enum class ThrottleMode {
  FeedforwardOnly,  ///< Model-based (open-loop) throttle
  PIDOnly,          ///< Feedback (closed-loop) throttle
  Combined          ///< Feedforward + PID feedback
};

/**
 * @class MotionController2D
 * @ingroup control
 * @brief 2D motion controller for path following and vehicle control.
 *
 * This class implements a flexible 2D motion controller for robotic vehicles,
 * supporting both model-based (feedforward) and feedback (PID) control for
 * speed (throttle) and steering. It is designed for path following, smooth
 * acceleration/deceleration, and robust integration with IMU and odometry
 * sensors.
 *
 * **Features:**
 * - Configurable PID controllers for both speed (throttle) and steering angle
 * correction.
 * - Multiple throttle control strategies: Feedforward (model-based), PID
 * (feedback), or Combined (feedforward + feedback).
 * - Path following with a sequence of waypoints, including automatic waypoint
 * advancement and goal detection.
 * - Smooth acceleration and deceleration profiles based on distance to target
 * and from start.
 * - Dynamic time step (dt) initialization for PID controllers based on update
 * frequency.
 * - Modular design for use with different vehicle types, IMU sensors, and
 * coordinate systems.
 * - Integration with the TinyRobotics message system for sending throttle and
 * steering commands.
 * - Customizable goal callback for user-defined actions upon reaching the final
 * waypoint.
 *
 * **Usage Example:**
 * @code
 *   MotionController2D<> controller(motionState, 10.0f, 2.0f);
 *   controller.setPath(path);
 *   controller.setThrottleMode(ThrottleMode::Combined);
 *   controller.begin();
 *   // In main loop:
 *   controller.update();
 * @endcode
 *
 * **Integration:**
 * - Use as a standalone controller for path following and vehicle control.
 * - Integrate with a message bus to receive and send control messages.
 * - Extend or customize by overriding protected methods or providing custom
 * callbacks.
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
    // Conservative initial PID values for easier tuning
    // kp=0.1, ki=0, kd=0
    configureSteeringPID(-30.0f, 30.0f, 1.0f, 0.0f, 0.0f);
    // kp=2.0, ki=0, kd=0
    configureSpeedPID(0.0f, 100.0f, 2.0f, 0.0f, 0.0f);
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
  void configureSpeedPID(float minOut, float maxOut, float kp, float ki,
                         float kd) {
    pidSpeed_.begin(0.0f, minOut, maxOut, kp, ki, kd);
  }

  /**
   * @brief Configure the PID controller for steering.
   * @param minOut Minimum output
   * @param maxOut Maximum output
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void configureSteeringPID(float minOut, float maxOut, float kp, float ki,
                            float kd) {
    pidSteering_.begin(0.0f, minOut, maxOut, kp, ki, kd);
  }

  /**
   * @brief Defines the path to follow as a sequence of waypoints
   */
  void setPath(Path<Coordinate<DistanceM>> path) { this->path = path; }

  /**
   * @brief Adds a single waypoint to the path (appended to the end)
   */
  void addWaypoint(Coordinate<DistanceM> target) {
    this->path.addWaypoint(target);
  }

  /**
   * @brief Set the target accuracy (meters) for reaching waypoints.
   * @param accuracyM The new target accuracy in meters.
   */
  void setTargetAccuracy(float accuracyM) { targetAccuracyM = accuracyM; }

  /**
   * @brief Start the controller and initialize state
   */
  bool begin() {
    is_active = true;
    has_distance = false;
    updateCount = 0;
    updateStartTimeMs = 0;
    updateEndTimeMs = 0;
    dtSetFromUpdates = false;

    if (!hasStartCoordinate) {
      startCoordinate = motionStateSource.getPosition();
      hasStartCoordinate = true;
    }
    return true;
  }

  /**
   * @brief Stop the controller and the vehicle
   */
  void end() { is_active = false; }

  /**
   * @brief Main control loop to be called periodically (e.g., in a timer or
   * main loop)
   */
  void update() {
    if (!is_active) return;
    if (!initializeDtFromUpdates()) {
      return;
    }
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

    desiredSpeedKmh =
        getDesiredSpeed(distanceFromStartM, distanceToTargetM, currentSpeedKmh);

    // Compute actuator commands and update class state
    resultThrottlePercent = getThrottlePercent(currentSpeedKmh);
    // ROS convention: positive steering = left turn. Invert heading error sign
    // so negative error (target left) yields negative steering (right turn).
    float pidInput = !isInverted ? -headingError : headingError;
    resultSteeringAngleDeg = pidSteering_.calculate(0.0f, pidInput);

    // Log summary (moved from sendControlMessages)
    TRLogger.info(
        "MotionController2D: distanceToTarget=%.2f m, desiredHeading=%.1f deg, "
        "currentHeading=%.1f deg, headingError=%.1f deg, steeringAngle=%.1f "
        "deg, desiredSpeed=%.2f km/h, currentSpeed=%.2f km/h, throttle=%.1f%%",
        distanceToTargetM, desiredHeading, currentHeading, headingError,
        resultSteeringAngleDeg, desiredSpeedKmh, currentSpeedKmh,
        resultThrottlePercent);

    // Send messages with computed values
    sendControlMessages(resultThrottlePercent, resultSteeringAngleDeg);

    // We have a valid distance to target, so we can compute control commands
    has_distance = true;
  }

  /**
   * @brief Set a custom callback to be called when reaching the goal. The
   * callback should return true if it handled the goal action, or false to
   * allow default handling.
   */
  void setOnGoalCallback(bool (*callback)(void*), void* ref = nullptr) {
    onGoalCallback = callback;
    if (ref != nullptr) onGoalRef = ref;
  }

  /**
   * @brief Get the last computed throttle percent.
   */
  float getThrottlePercent() const { return resultThrottlePercent; }
  /**
   * @brief Get the last computed steering angle in degrees.
   */
  float getSteeringAngleDeg() const { return resultSteeringAngleDeg; }
  /**
   * @brief Get the last computed steering angle as an Angle object.
   */
  Angle getSteeringAngle() const {
    return Angle(resultSteeringAngleDeg, AngleUnit::DEG);
  }

  /**
   * @brief Get the Target Accuracy object
   *
   * @param unit
   * @return float
   */
  float getTargetAccuracy(DistanceUnit unit) const {
    return Distance(targetAccuracyM, DistanceUnit::M).getValue(unit);
  }

  /**
   * @brief Returns true if the goal has been reached
   */
  bool isGoalReached() const {
    if (!has_distance) return false;
    return distanceToTargetM < targetAccuracyM;
  }

  /**
   * @brief Set the throttle control mode.
   * @param mode The throttle control strategy to use (FeedforwardOnly, PIDOnly,
   * Combined).
   */
  void setThrottleMode(ThrottleMode mode) { throttleMode = mode; }

  /**
   * @brief Set the Inverted Steering logic: by default we use ROS logic for
   * steering correction (positive angle = turn left), but some vehicles may
   * require the opposite. Set to true to invert the steering direction.
   * @param inverted
   */
  void setInvertedSteering(bool inverted) { isInverted = inverted; }

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
  void* onGoalRef = this;
  float resultThrottlePercent = 0.0f;
  float resultSteeringAngleDeg = 0.0f;
  /// Throttle control mode
  ThrottleMode throttleMode = ThrottleMode::Combined;
  int updateCount = 0;
  unsigned long updateStartTimeMs = 0;
  unsigned long updateEndTimeMs = 0;
  bool dtSetFromUpdates = false;
  bool isInverted = false;  // For steering direction inversion if needed

  /// Calculate desired speed based on distance to target and start
  float getDesiredSpeed(float distanceFromStartM, float distanceToTarget,
                        float currentSpeedKmh) {
    // Compute a desired speed profile: slow down as you approach the target
    float minSpeedKmh = 0.0f;
    float desiredSpeedKmh =
        maxSpeedKmh;  // * (distanceToTarget / accelDistanceM);
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
    return desiredSpeedKmh;
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
        TRLogger.info(
            "Reached waypoint at (%.2f m, %.2f m), distance to target: %.2f m",
            targetPos.x, targetPos.y, distanceToTargetM);
        path.removeHead();
        if (path.isEmpty()) {
          TRLogger.info("Reached final goal!");
          if (onGoalCallback) {
            onGoalCallback(onGoalRef);
          }
          return false;
        }
        targetPos = path[0];
        return false;
      }
    }
    return true;
  }

  /// Compute angle difference range [-180, 180]
  float computeHeadingError(float desiredHeading, float currentHeading) {
    float headingError = desiredHeading - currentHeading;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    return headingError;
  }

  /// Model-based feedforward throttle estimate: 100% throttle = maxSpeedKmh
  float feedforwardThrottle(float desiredSpeedKmh) const {
    if (maxSpeedKmh <= 0.0f) return 0.0f;
    return 100.0f * desiredSpeedKmh / maxSpeedKmh;
  }

  /// Compute the throttle percent based on the selected mode.
  float getThrottlePercent(float currentSpeedKmh) {
    float speedError = desiredSpeedKmh - currentSpeedKmh;
    float ff = feedforwardThrottle(desiredSpeedKmh);
    float fb = pidSpeed_.calculate(0.0f, -speedError);
    float throttlePercent = 0.0f;
    switch (throttleMode) {
      case ThrottleMode::FeedforwardOnly:
        throttlePercent = ff;
        break;
      case ThrottleMode::PIDOnly:
        throttlePercent = fb;
        break;
      case ThrottleMode::Combined:
      default:
        throttlePercent = ff + fb;
        break;
    }
    // Clamp throttle to [0, 100]
    if (throttlePercent > 100.0f) throttlePercent = 100.0f;
    if (throttlePercent < 0.0f) throttlePercent = 0.0f;
    if (throttlePercent > 0.0f && throttlePercent < 1.0f)
      throttlePercent = 1.0f;  // avoid too low throttle
    return throttlePercent;
  }

  /// Send control messages for throttle and steering angle
  void sendControlMessages(float throttlePercent, float steeringAngleDeg) {
    Message<float> msg;
    msg.source = MessageOrigin::Autonomy;
    msg.content = MessageContent::Throttle;
    msg.unit = Unit::Percent;
    msg.value = throttlePercent;
    sendMessage(msg);

    msg.content = MessageContent::SteeringAngle;
    msg.unit = Unit::AngleDegree;
    msg.value = steeringAngleDeg;
    sendMessage(msg);
  }

  /// Handles dt initialization from first 10 updates
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
      // Reconfigure both PIDs with new dt, keep other params
      pidSpeed_.setDt(dt);
      pidSteering_.setDt(dt);
      dtSetFromUpdates = true;
      TRLogger.info("Initialized PID dt from updates: %.3f seconds (%.1f ms)",
                    dt, avgMs);
    }
    return dtSetFromUpdates;
  }
};

}  // namespace tinyrobotics