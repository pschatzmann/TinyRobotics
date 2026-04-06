#pragma once
#include "FastAccelStepper.h"
#include "TinyRobotics/utils/Config.h"
#ifdef AVR
#include "AVRStepperPins.h"  // Only required for AVR controllers
#endif
#include "Motor.h"

namespace tinyrobotics {

/**
 * @class StepperMotor
 * @ingroup motors
 * @brief Stepper motor driver using FastAccelStepper for Arduino.
 *
 * This class provides an interface to control stepper motors with acceleration
 * and speed control. It supports:
 *   - Continuous speed mode: setSpeed(percent) for indefinite movement at a
 * given speed and direction.
 *   - Relative movement: moveRevolutions(revolutions, speedPercent, callback,
 * ref) to move a specific number of revolutions, with optional callback on
 * completion.
 *   - Distance-based movement: moveDistance(distance, wheelDiameter,
 * speedPercent, callback, ref) to move a specific physical distance (using
 * wheel diameter), with optional callback.
 *
 * You can set pins, maximum speed, acceleration, and steps per revolution. The
 * motor can be started, stopped, or moved by distance or revolutions.
 * Acceleration is handled automatically based on the configured accelerationMs.
 *
 * The class uses FastAccelStepper's continuous mode for setSpeed(), and
 * supports asynchronous notification via callback for move operations.
 *
 * @dependencies
 * - Requires the FastAccelStepper library:
 * https://github.com/ESP32DIYer/FastAccelStepper (Install via Arduino Library
 * Manager or PlatformIO)
 * - Uses Arduino core functions (e.g., constrain, map)
 *
 * Main methods:
 * - setPins(stepPin, dirPin, enablePin): Assigns the control pins for the
 * stepper driver.
 * - setMaxSpeed(stepsPerSec): Sets the maximum speed in steps per second.
 * - setStepsPerRevolution(steps): Sets the number of steps per full revolution.
 * - setAccelerationMs(ms): Sets the time in milliseconds to reach max speed
 * (acceleration).
 * - begin(): Initializes the stepper hardware and prepares for movement.
 * - setSpeed(percent): Starts continuous movement at the given speed percentage
 * (-100 to 100) using continuous mode.
 * - moveRevolutions(revolutions, speedPercent, callback, ref): Move the stepper
 * by a given number of revolutions at a specified speed, with optional
 * callback.
 * - moveDistance(distance, wheelDiameter, speedPercent, callback, ref): Move
 * the stepper by a physical distance (using wheel diameter), at a specified
 * speed, with optional callback.
 * - end(): Stops the stepper motor.
 *
 * Usage notes:
 * - Call begin() after setting pins and speed parameters.
 * - Use setSpeed(percent) to start or change continuous movement. Use
 * setSpeed(0) to stop.
 * - Acceleration is set in setSpeed() based on the configured accelerationMs.
 *
 * Example usage:
 * @code
 * Stepper stepper;
 * stepper.setPins(2, 3, 4);
 * stepper.setMaxSpeed(1000);
 * stepper.setStepsPerRevolution(200);
 * stepper.setAccelerationMs(2000);
 * stepper.begin();
 * stepper.setSpeed(50); // Move at 50% of max speed (continuous mode)
 * stepper.setSpeed(0);  // Stop
 * stepper.end();        // Ensure motor is stopped
 * @endcode
 */
template <typename T = float>
class StepperMotor : public Motor<T> {
 public:
  StepperMotor(uint8_t id = 0) { setID(id); }
  StepperMotor(uint8_t id, uint16_t stepsPerRevolution,
               float maxSpeedStepsPerSec, uint16_t accelerationMs) {
    setID(id);
    setMaxSpeed(maxSpeedStepsPerSec);
    setAccelerationMs(accelerationMs);
    setStepsPerRevolution(stepsPerRevolution);
  }

  void setPins(int stepPin, int dirPin, int enablePin = -1) {
    pinStep = stepPin;
    pinDir = dirPin;
    pinEnable = enablePin;
    is_pins_set = true;
  }

  /// Set maximum speed in steps per second (e.g., 500 for 1.8 degree stepper,
  /// 1000
  void setMaxSpeed(float stepsPerSec) { maxSpeedHz = stepsPerSec; }

  /// Set acceleration time in milliseconds to reach max speed (e.g., 2000 ms)
  void setAccelerationMs(uint16_t ms) { accelerationMs = ms; }

  /// Set the number of steps per full revolution (e.g., 200 for 1.8 degree
  /// stepper)
  void setStepsPerRevolution(int steps) { stepsPerRevolution = steps; }

  /// Initialize the stepper hardware. Must be called after setting pins and
  /// speed parameters.
  bool begin() {
    if (!is_pins_set) return false;
    engine.init();
    stepper = engine.stepperConnectToPin(pinStep);
    if (stepper) {
      stepper->setDirectionPin(pinDir);
      if (pinEnable != -1) stepper->setEnablePin(pinEnable);
      stepper->setAutoEnable(true);
    }
    return stepper != nullptr;
  }

  /// Set continuous speed as a percentage (-100 to 100). Positive for forward,
  /// negative for backward.
    // Set value as percentage (-100 to 100)
    bool setValuePercent(T percent) override {
      if (!stepper) return false;
      lastValuePercent = constrain(percent, -100.0f, 100.0f);
      setSpeedPercent(lastValuePercent);
      if (lastValuePercent >= 0) {
        stepper->runForward();
      } else {
        stepper->runBackward();
      }
      return true;
    }

    T getValuePercent() const override {
      return lastValuePercent;
    }

  //   /// Move the stepper by n revolutions (positive or negative) and
  //   optionally
  //   /// call a callback when done.
  //   bool moveRevolutions(T revolutions, int8_t speedPercent,
  //                        void* ref = nullptr) {
  //     if (!stepper) return false;
  //     setSpeedPercent(speedPercent);
  //     int steps = revolutions * stepsPerRevolution;
  //     this->callback = callback;
  //     this->ref = ref;
  //     stepper->setNotifyCallback(callbackFastAccel);
  //     stepper->move(steps);
  //   }

  //   /// Move the stepper by a specific distance (in units of length) based on
  //   the bool moveDistance(Distance distance, Distance wheelDiameter,
  //                     int8_t speedPercent, void (*callback)(void*) = nullptr,
  //                     void* ref = nullptr) {
  //     if (!stepper) return false;
  //     T circumferenceM = PI * wheelDiameter.getDistance(DistanceUnit::M);
  //     T revolutions = distance.getDistance(DistanceUnit::M) /
  //     circumferenceM; return moveRevolutions(revolutions, speedPercent,
  //     callback, ref);
  //   }

  /// Stop the stepper motor.
  void end() override {
    if (stepper) {
      stepper->stopMove();
    }
  }

  bool isPinsSet() const override { return is_pins_set; }

 protected:
  T lastValuePercent = 0.0f;
  FastAccelStepperEngine engine;
  FastAccelStepper* stepper = NULL;
  int pinStep = -1;
  int pinDir = -1;
  int pinEnable = -1;
  bool is_pins_set = false;
  // steps/sec: typical values: 500 for 1.8 degree stepper, 1000 for 0.9 degree
  // stepper, etc.
  float maxSpeedHz = 2000;
  // time in ms to reach max speed, adjust as needed
  uint16_t accelerationMs = 2000;
  // typical values: 200 for 1.8 degree stepper, 400 for 0.9 degree stepper,
  // etc.
  uint16_t stepsPerRevolution = 200;
  void* ref = nullptr;
  void (*callback)(void*) = nullptr;

  void callbackFastAccel(FastAccelStepper* s) {
    if (callback) {
      callback(ref);
    }
  }
  
  void setSpeedPercent(T percent) {
    percent = constrain(percent, -100.0f, 100.0f);
    int speed = map((int)percent, -100, 100, -maxSpeedHz, maxSpeedHz);

    // Calculate acceleration in steps/sec^2 based on the time to reach max
    // speed
    int acceleration = (maxSpeedHz * 1000) / accelerationMs;  // steps/sec^2
    stepper->setAcceleration(acceleration);

    // set speed (FastAccelStepper uses setSpeedInHz for continuous mode)
    stepper->setSpeedInHz(abs(speed));
  }

};
}  // namespace tinyrobotics
