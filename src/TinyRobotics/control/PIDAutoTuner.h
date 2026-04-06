#pragma once
#include <math.h>
#include <stdint.h>

namespace tinyrobotics {

/**
 * @class PIDAutoTuner
 * @ingroup control
 * @brief Automatic PID tuning using the relay (Åström-Hägglund) method.
 *
 * This class implements an automatic PID tuning algorithm based on the relay
 * (bang-bang) method, also known as the Åström-Hägglund relay auto-tuning
 * technique. It applies a relay (on/off) control to the system and analyzes the
 * resulting oscillations to estimate the ultimate gain (Ku) and period (Tu).
 * These values are then used to compute PID parameters using the
 * Ziegler-Nichols tuning rules.
 *
 * - The user specifies a target setpoint, relay amplitude, and sample time
 * (dt).
 * - The tuner applies a relay output and detects peaks in the process variable.
 * - After sufficient oscillations, it calculates Ku, Tu, and recommended PID
 * gains.
 * - The process can be monitored with isFinished() and the results retrieved
 * with getResult().
 * - Hysteresis can be adjusted to avoid chattering.
 *
 * **Usage Example:**
 * @code
 *   PIDAutoTuner tuner(target, relayAmp, dt);
 *   while (!tuner.isFinished()) {
 *     float control = tuner.update(measuredValue);
 *     // Apply 'control' to actuator
 *   }
 *   auto result = tuner.getResult();
 *   if (result.success) {
 *     // Use result.Kp, result.Ki, result.Kd
 *   }
 * @endcode
 *
 * @see setHysteresis, getResult, isFinished, update, reset
 *
 * @author Phil Schatzmann
 */

class PIDAutoTuner {
 public:
  struct Result {
    float Kp;
    float Ki;
    float Kd;
    float Ku;
    float Tu;
    bool success;
  };

  PIDAutoTuner(float target, float relayAmp, float dt)
      : target(target), relayAmp(relayAmp), dt(dt) {}

  void reset() {
    state = State::INIT;
    peakCount = 0;
    lastY = 0;
    lastPeakTime = 0;
    maxY = -1e9;
    minY = 1e9;
    time = 0;
    output = relayAmp;
    success = false;
  }

  float update(float y) {
    time += dt;

    // Relay control (bang-bang)
    if (y > target + hysteresis)
      output = -relayAmp;
    else if (y < target - hysteresis)
      output = relayAmp;

    detectPeaks(y);

    return output;
  }

  bool isFinished() const { return success; }

  Result getResult() const {
    Result r{};
    r.success = success;

    if (!success) return r;

    float amplitude = (avgMax - avgMin) / 2.0f;

    // Ultimate gain (relay method)
    float Ku = (4.0f * relayAmp) / (static_cast<float>(M_PI) * amplitude);

    float Tu = avgPeriod;

    r.Ku = Ku;
    r.Tu = Tu;

    // Ziegler-Nichols PID tuning
    r.Kp = 0.6f * Ku;
    r.Ki = 2.0f * r.Kp / Tu;
    r.Kd = r.Kp * Tu / 8.0f;

    return r;
  }

  void setHysteresis(float h) { hysteresis = h; }

 protected:
  enum class State { INIT, RUNNING };

  float target;
  float relayAmp;
  float dt;
  float hysteresis = 0.01f;

  float output = 0;
  float time = 0;

  float lastY = 0;

  float maxY = -1e9;
  float minY = 1e9;

  float avgMax = 0;
  float avgMin = 0;
  float avgPeriod = 0;

  float lastPeakTime = 0;

  int peakCount = 0;
  bool lastWasMax = false;

  bool success = false;

  State state = State::INIT;

  void detectPeaks(float y) {
    // Track extrema
    if (y > maxY) maxY = y;
    if (y < minY) minY = y;

    // Detect zero crossing slope change
    if ((lastY < y) && (lastWasMax == false)) {
      // rising
    } else if ((lastY > y) && (lastWasMax == false)) {
      // max peak
      registerPeak(true);
      lastWasMax = true;
    } else if ((lastY > y) && (lastWasMax == true)) {
      // falling
    } else if ((lastY < y) && (lastWasMax == true)) {
      // min peak
      registerPeak(false);
      lastWasMax = false;
    }

    lastY = y;
  }

  void registerPeak(bool isMax) {
    peakCount++;

    float period = time - lastPeakTime;
    lastPeakTime = time;

    // Ignore first few peaks
    if (peakCount < 4) return;

    // Running averages
    if (isMax)
      avgMax = 0.8f * avgMax + 0.2f * maxY;
    else
      avgMin = 0.8f * avgMin + 0.2f * minY;

    avgPeriod = 0.8f * avgPeriod + 0.2f * period;

    // Reset extrema
    maxY = -1e9;
    minY = 1e9;

    // Finish after enough stable oscillations
    if (peakCount > 12) {
      success = true;
    }
  }
};

}  // namespace tinyrobotics