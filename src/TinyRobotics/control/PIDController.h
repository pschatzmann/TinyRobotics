#pragma once

#include <cmath>
#include <assert.h>

namespace tinyrobotics {

/**
 * @class PIDController
 * @ingroup control
 * @brief Implements a simple, header-only Proportional-Integral-Derivative
 * (PID) controller.
 *
 * This class provides a basic PID controller for closed-loop control of
 * processes such as motors, temperature, position, and more. The controller
 * computes a control output based on the error between a target setpoint and a
 * measured process value, using configurable proportional, integral, and
 * derivative gains.
 *
 * ## Features
 * - Simple API: `begin()` to configure, `calculate()` to compute control output
 * - Supports output clamping (min/max)
 * - Suitable for real-time and embedded systems
 * - All state is internal; no dynamic memory allocation
 *
 * ## Usage Example
 * @code
 *   PIDController pid;
 *   pid.begin(0.01, 255, 0, 1.0, 0.5, 0.1); // dt, max, min, kp, ki, kd
 *   N control = pid.calculate(target, measured);
 * @endcode
 *
 * ## Parameters
 * - dt: Loop interval time (seconds)
 * - max: Maximum output value
 * - min: Minimum output value
 * - kp: Proportional gain
 * - ki: Integral gain
 * - kd: Derivative gain
 *
 * ## Methods
 * - begin(dt, max, min, kp, ki, kd): Initialize controller parameters
 * - calculate(target, measured): Compute control output for current error
 *
 * ## Applications
 * - Motor speed and position control
 * - Temperature regulation
 * - Robotics and automation
 * - Any process requiring feedback control
 *
 * @author Phil Schatzmann
 */

template <class N = float>
class PIDController {
 public:
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  // kp -  proportional gain
  // ki -  Integral gain
  // kd -  derivative gain
  bool begin(N dt, N max, N min, N kp, N ki, N kd) {
    this->dt = dt;
    this->max = max;
    this->min = min;
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    return true;
  }

  // target = desired process value
  // measured = current process value:
  // returns new process value
  N calculate(N target, N measured) {
    // Calculate errori
    N error = target - measured;

    // Proportional term
    N pout = kp * error;

    // Interal term
    integral += error * dt;
    N Iout = ki * integral;

    // Derivative term
    assert(dt != 0.0);

    N derivative = (error - preerror) / dt;
    N dout = kd * derivative;

    // Calculate total output
    N output = pout + Iout + dout;

    // Restrict to max/min
    if (output > max)
      output = max;
    else if (output < min)
      output = min;

    // Save error to previous error
    preerror = error;

    return output;
  }

 protected:
  N dt = 1.0f;
  N max = 0.0f;
  N min = 0.0f;
  N kp = 0.0f;
  N kd = 0.0f;
  N ki = 0.0f;
  N preerror = 0.0f;
  N integral = 0.0f;
};

}  // namespace tinyrobotics
