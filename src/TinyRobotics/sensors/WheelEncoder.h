#pragma once
#include <cmath>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/control/Scheduler.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class WheelEncoder
 * @brief Measures wheel rotation and computes distance and speed using encoder
 * ticks for mobile robots.
 *
 * The WheelEncoder class provides an interface for tracking the movement of a
 * robot's wheel using an incremental encoder. It accumulates encoder ticks to
 * estimate the distance traveled and calculates speed based on tick timing.
 *
 * Key features:
 * - Configurable wheel diameter and ticks per revolution for accurate distance
 * estimation.
 * - Periodic reporting of distance and speed via the MessageSource interface.
 * - Slip calibration support to compensate for wheel slip or surface effects.
 * - Designed for use with interrupt-driven tick updates (call setTick() in your
 * ISR).
 *
 * Usage:
 * 1. Create a WheelEncoder instance and configure the wheel diameter and ticks
 * per revolution.
 * 2. Call begin() to reset and start periodic reporting.
 * 3. In your encoder interrupt handler, call setTick() to update the encoder
 * state.
 * 4. Use getDistanceM(), getSpeedMPS(), or getDistance() to retrieve odometry
 * data.
 * 5. Optionally, calibrate slip using calibrateSlip() if you observe systematic
 * odometry errors.
 *
 * Example:
 * @code
 * WheelEncoder encoder;
 * encoder.setWheelDiameter(0.065); // 65mm wheel
 * encoder.setTicksPerRevolution(20);
 * encoder.begin();
 * // In your interrupt:
 * encoder.setTick();
 * // In your main loop:
 * float distance = encoder.getDistanceM();
 * float speed = encoder.getSpeedMPS();
 * @endcode
 *
 * This class is intended for embedded robotics applications (Arduino, ESP32,
 * etc.) and integrates with the TinyRobotics messaging framework.
 */
class WheelEncoder : public MessageSource {
 public:
  /**
   * @brief Default constructor. Wheel diameter and ticks per revolution must be
   * set before use.
   */
  WheelEncoder() = default;

  /**
   * @brief Constructor with wheel diameter and ticks per revolution.
   * @param wheelDiameterM The diameter of the wheel (Distance object).
   * @param ticksPerRevolution Number of encoder ticks per full wheel
   * revolution.
   */
  WheelEncoder(Distance wheelDiameterM, int ticksPerRevolution = 1)
      : WheelEncoder() {
    setWheelDiameter(wheelDiameterM);
    setTicksPerRevolution(ticksPerRevolution);
  }

  /**
   * @brief Set the wheel diameter using a Distance object.
   * @param wheelDiameter The wheel diameter.
   */
  void setWheelDiameter(Distance wheelDiameter) {
    wheelDiameterM = wheelDiameter.getValue(DistanceUnit::M);
  }

  /**
   * @brief Set the wheel diameter using a float value and unit.
   * @param diameter The wheel diameter value.
   * @param unit The unit of the diameter (default: meters).
   */
  void setWheelDiameter(float diameter, DistanceUnit unit = DistanceUnit::M) {
    setWheelDiameter(Distance(diameter, unit));
  }

  /**
   * @brief Set the number of encoder ticks per wheel revolution.
   * @param ticks Number of ticks per revolution.
   */
  void setTicksPerRevolution(int ticks) {
    ticksPerRevolution = ticks;
    distancePerTickM = M_PI * wheelDiameterM / ticksPerRevolution;
  }

  /**
   * @brief Set the reporting frequency for distance/speed messages.
   * @param ms Reporting interval in milliseconds.
   */
  void setReportingFrequencyMs(uint16_t ms) {
    reportingScheduler.begin(ms, sendMessageCB, this);
  }

  /**
   * @brief Get the total distance traveled since last reset.
   * @return Distance object representing the traveled distance in meters.
   */
  Distance getDistance() const { return Distance(distanceM, DistanceUnit::M); }

  /**
   * @brief Get the total distance traveled in meters.
   * @return Distance in meters.
   */
  float getDistanceM() const { return distanceM * slipFactor; }

  /**
   * @brief Get the total distance traveled in the specified unit.
   * @param unit The distance unit.
   * @return Distance in the specified unit.
   */
  float getDistance(DistanceUnit unit) const {
    Distance dist(distanceM, DistanceUnit::M);
    return dist.getValue(unit);
  }

  /**
   * @brief Get the distance corresponding to a given number of ticks (in
   * meters).
   * @param ticks Number of encoder ticks.
   * @return Distance in meters.
   */
  float getDistanceForTicksM(size_t ticks) const {
    return ticks * distancePerTickM;
  }

  /**
   * @brief Get the distance for a given number of ticks in the specified unit.
   * @param ticks Number of encoder ticks.
   * @param unit The distance unit.
   * @return Distance in the specified unit.
   */
  float getDistanceForTicks(size_t ticks, DistanceUnit unit) const {
    Distance dist(getDistanceForTicksM(ticks), DistanceUnit::M);
    return dist.getValue(unit);
  }

  /**
   * @brief Calculate speed in meters per second based on distance traveled
   * since last calculation and elapsed time.
   * @return Speed in meters per second.
   */
  float getSpeedMPS() {
    float speedMPS = 0;
    uint32_t currentTimeMs = millis();
    uint32_t elapsedTimeMs = currentTimeMs - lastSpeedCalcTimeMs;
    if (elapsedTimeMs > 0) {
      float distanceTraveledM = (distanceM - lastDistanceM) * slipFactor;
      speedMPS = distanceTraveledM / (elapsedTimeMs / 1000.0);
      lastSpeedCalcTimeMs = currentTimeMs;
      lastDistanceM = distanceM;
    }
    return speedMPS;
  }

  /**
   * @brief Resets the encoder counts and distance.
   *
   * Also starts periodic reporting if not already active.
   * @return true if the encoder is properly configured (wheel diameter and
   * ticks per revolution are set).
   */
  bool begin() {
    distanceM = 0;
    lastSpeedCalcTimeMs = millis();
    // Default to 1 second reporting if not already set
    if (!reportingScheduler.isActive()) {
      setReportingFrequencyMs(1000);
    }
    return wheelDiameterM > 0 && ticksPerRevolution > 0;
  }

  /**
   * @brief To be called by the pin interrupt handler when a tick is detected.
   *
   * Updates the distance and triggers reporting if needed.
   */
  void setTick() {
    distanceM += distancePerTickM;
    reportingScheduler.run();
  }

  /**
   * @brief Trigger sending messages for distance and speed.
   *
   * Sends the current distance and speed as messages.
   */
  void sendMessage() {
    Message<float> msg(MessageContent::Distance, getDistanceM(), Unit::Meters);
    msg.source = MessageOrigin::Sensor;
    MessageSource::sendMessage(msg);

    Message<float> speedMsg(MessageContent::Speed, getSpeedMPS(),
                            Unit::MetersPerSecond);
    speedMsg.source = MessageOrigin::Sensor;
    MessageSource::sendMessage(speedMsg);
  }

  void end() { reportingScheduler.end(); }

  /**
   * @brief Set the slip correction factor.
   *
   * The slip factor is used to correct encoder-based distance and speed
   * estimates. It should be set to (actual_distance / encoder_distance) after
   * calibration. Default is 1.0 (no slip).
   * @param slipFactor The slip correction factor (typically <= 1.0).
   */
  void setSlipFactor(float slipFactor) { this->slipFactor = slipFactor; }

  /**
   * @brief Calibrate slip by providing a known actual distance.
   *
   * Call this after moving the robot a known distance and accumulating encoder
   * ticks. The slip factor will be set to (actual_distance / encoder_distance).
   * @param actualDistanceM The actual distance traveled in meters.
   */
  void calibrateSlip(float actualDistanceM) {
    float encoderDistance = getRawDistanceM();
    if (encoderDistance > 0) {
      slipFactor = actualDistanceM / encoderDistance;
    }
  }

  /**
   * @brief Get the slip correction factor.
   * @return The slip factor (1.0 = no slip, <1.0 = slip present).
   */
  float getSlipFactor() const { return slipFactor; }

  /**
   * @brief Get the raw (uncorrected) distance in meters from the encoder.
   * @return Raw encoder distance in meters.
   */
  float getRawDistanceM() const { return distanceM; }

 protected:
  volatile float distanceM = 0.0f;
  float wheelDiameterM = 0.0f;
  float distancePerTickM = 0.0f;
  float lastDistanceM = 0.0f;
  float slipFactor = 1.0f;
  uint32_t ticksPerRevolution = 0;
  uint32_t lastSpeedCalcTimeMs = 0;
  Scheduler reportingScheduler;

  /**
   * @brief Static callback for the reporting scheduler to send messages.
   * @param ref Pointer to the WheelEncoder instance.
   */
  static void sendMessageCB(void* ref) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(ref);
    encoder->sendMessage();
  }
};

}  // namespace tinyrobotics
