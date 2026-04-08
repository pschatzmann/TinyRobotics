#pragma once
#include <cmath>
#include <vector>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/control/Scheduler.h"
#include "TinyRobotics/odometry/ISpeedSource.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class WheelEncoder
 * @ingroup sensors
 * @brief Measures wheel rotation and computes per-wheel distance and speed
 * using encoder ticks for mobile robots.
 *
 * The WheelEncoder class provides a multi-wheel, vectorized interface for
 * tracking the movement of a robot's wheels using incremental encoders. It
 * accumulates encoder ticks for each wheel to estimate the distance traveled
 * and calculates speed based on tick timing, supporting robust odometry for
 * differential, skid-steer, and multi-motor vehicles.
 *
 * Key features:
 * - Supports any number of wheels (configurable at construction).
 * - Vectorized state for distance, speed, and tick timing per wheel.
 * - Interface-compliant with ISpeedSource for modular odometry integration.
 * - Configurable wheel diameter and ticks per revolution for accurate distance
 * estimation.
 * - Periodic reporting of distance and speed via the MessageSource interface.
 * - Slip calibration support to compensate for wheel slip or surface effects.
 * - Designed for use with interrupt-driven tick updates (call setTick() in your
 * ISR, specifying the wheel index).
 *
 * Usage:
 * 1. Create a WheelEncoder instance, specifying the number of wheels if needed,
 * and configure the wheel diameter and ticks per revolution.
 * 2. Call begin() to reset and start periodic reporting.
 * 3. In your encoder interrupt handler, call setTick(motor) to update the
 * encoder state for the correct wheel.
 * 4. Use getDistanceM(motor), getSpeedMPS(motor), or getDistance(unit, motor)
 * to retrieve odometry data for each wheel.
 * 5. Optionally, calibrate slip using calibrateSlip() if you observe systematic
 * odometry errors.
 *
 * Example:
 * @code
 * WheelEncoder encoder(2); // Two wheels (differential drive)
 * encoder.setWheelDiameter(0.065); // 65mm wheel
 * encoder.setTicksPerRevolution(20);
 * encoder.begin();
 * // In your interrupt:
 * encoder.setTick(0); // Left wheel
 * encoder.setTick(1); // Right wheel
 * // In your main loop:
 * float leftDistance = encoder.getDistanceM(0);
 * float rightDistance = encoder.getDistanceM(1);
 * float leftSpeed = encoder.getSpeedMPS(0);
 * float rightSpeed = encoder.getSpeedMPS(1);
 * @endcode
 *
 * This class is intended for embedded robotics applications (Arduino, ESP32,
 * etc.) and integrates with the TinyRobotics messaging framework. It is
 * suitable for use as a modular speed/distance source in extensible odometry
 * pipelines.
 */
class WheelEncoder : public MessageSource, public ISpeedSource {
 public:
  /**
   * @brief Default constructor. Wheel diameter and ticks per revolution must be
   * set before use.
   */
  WheelEncoder(size_t numWheels = 1)
      : speedMPS(numWheels, 0.0f),
        distanceM(numWheels, 0.0f),
        lastDistanceM(numWheels, 0.0f),
        lastSpeedCalcTimeMs(numWheels, 0),
        numWheels(numWheels) {}

  /**
   * @brief Constructor with wheel diameter and ticks per revolution.
   * @param wheelDiameterM The diameter of the wheel (Distance object).
   * @param ticksPerRevolution Number of encoder ticks per full wheel
   * revolution.
   */
  WheelEncoder(Distance wheelDiameterM, int ticksPerRevolution = 1,
               size_t numWheels = 1)
      : WheelEncoder(numWheels) {
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
    distancePerTickM =
        static_cast<float>(M_PI) * wheelDiameterM / ticksPerRevolution;
  }

  /**
   * @brief Defines the reporint frequency for distance and speed messages.
   * @param ms Reporting interval in milliseconds.
   */
  void setReportingFrequencyMs(uint16_t ms) {
    reportingScheduler.begin(ms, sendMessageCB, this);
  }

  /**
   * @brief Get the total distance traveled since last reset.
   * @return Distance object representing the traveled distance in meters.
   */
  // Removed: getDistance() returning Distance from a vector (invalid)

  /**
   * @brief Get the total distance traveled in meters.
   * @return Distance in meters.
   */
  float getDistanceM(size_t motor = 0) const {
    if (motor >= numWheels) return 0.0f;
    return distanceM[motor] * slipFactor;
  }

  /**
   * @brief Get the total distance traveled in the specified unit.
   * @param unit The distance unit.
   * @return Distance in the specified unit.
   */
  float getDistance(DistanceUnit unit, size_t motor = 0) const {
    Distance dist(getDistanceM(motor), DistanceUnit::M);
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
   * @brief Resets the encoder counts and distance.
   *
   * Also starts periodic reporting if not already active.
   * @return true if the encoder is properly configured (wheel diameter and
   * ticks per revolution are set).
   */
  bool begin() {
    distanceM.resize(numWheels, 0.0f);
    lastSpeedCalcTimeMs.resize(numWheels, 0);
    lastDistanceM.resize(numWheels, 0.0f);
    speedMPS.resize(numWheels, 0.0f);
    for (size_t i = 0; i < numWheels; ++i) {
      distanceM[i] = 0;
      lastSpeedCalcTimeMs[i] = millis();
      lastDistanceM[i] = 0;
      speedMPS[i] = 0;
    }
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
  void setTick(size_t motor = 0) {
    if (motor >= numWheels) return;
    distanceM[motor] += distancePerTickM;
    reportingScheduler.run();
  }

  Speed getSpeed(uint8_t motor = 0) const override {
    if (motor >= numWheels) return Speed(0.0f, SpeedUnit::MPS);
    return Speed(const_cast<WheelEncoder*>(this)->getSpeedMPS(motor),
                 SpeedUnit::MPS);
  }

  /// Just provide the last reported speed without inertia modeling

  Speed updateSpeed(uint32_t deltaTimeMs, uint8_t motor = 0) override {
    if (motor >= numWheels) return Speed(0.0f, SpeedUnit::MPS);
    return Speed(speedMPS[motor], SpeedUnit::MPS);
  }

  /**
   * @brief Trigger sending messages for distance and speed.
   *
   * Sends the current distance and speed as messages.
   */
  void sendMessage() {
    for (size_t i = 0; i < numWheels; ++i) {
      Message<float> msg(MessageContent::Distance, getDistanceM(i),
                         Unit::Meters);
      msg.origin = MessageOrigin::Sensor;
      MessageSource::sendMessage(msg);

      Message<float> speedMsg(MessageContent::Speed, getSpeedMPS(i),
                              Unit::MetersPerSecond);
      speedMsg.origin = MessageOrigin::Sensor;
      MessageSource::sendMessage(speedMsg);
    }
  }

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
  float getRawDistanceM(size_t motor = 0) const {
    if (motor >= numWheels) return 0.0f;
    return distanceM[motor];
  }

  /// Not used
  void setThrottlePercent(float value, uint8_t motor = 0) override {}

  size_t getMotorCount() const override { return numWheels; }

 protected:
  std::vector<float> speedMPS;
  std::vector<float> distanceM;
  // Returns the distance for a specific motor (default 0)
  Distance getDistance(size_t motor = 0) const {
    if (motor >= numWheels) return Distance(0.0f, DistanceUnit::M);
    return Distance(distanceM[motor], DistanceUnit::M);
  }
  float wheelDiameterM = 0.0f;
  float distancePerTickM = 0.0f;
  std::vector<float> lastDistanceM;
  float slipFactor = 1.0f;
  uint32_t ticksPerRevolution = 0;
  std::vector<uint32_t> lastSpeedCalcTimeMs;
  Scheduler reportingScheduler;
  size_t numWheels = 1;

  /**
   * @brief Static callback for the reporting scheduler to send messages.
   * @param ref Pointer to the WheelEncoder instance.
   */
  static void sendMessageCB(void* ref) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(ref);
    encoder->sendMessage();
  }

  /**
   * @brief Calculate speed in meters per second based on distance traveled
   * since last calculation and elapsed time.
   * @return Speed in meters per second.
   */
  float getSpeedMPS(size_t motor = 0) {
    if (motor >= numWheels) return 0.0f;
    speedMPS[motor] = 0;
    uint32_t currentTimeMs = millis();
    uint32_t elapsedTimeMs = currentTimeMs - lastSpeedCalcTimeMs[motor];
    if (elapsedTimeMs > 0) {
      float distanceTraveledM =
          (distanceM[motor] - lastDistanceM[motor]) * slipFactor;
      speedMPS[motor] = distanceTraveledM / (elapsedTimeMs / 1000.0f);
      lastSpeedCalcTimeMs[motor] = currentTimeMs;
      lastDistanceM[motor] = distanceM[motor];
    }
    return speedMPS[motor];
  }
};

}  // namespace tinyrobotics
