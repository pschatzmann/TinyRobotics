#pragma once

/**
 * @brief Abstract base class for all motor driver types.
 *
 * The Motor class defines a common interface for all motor drivers in the library.
 * It enforces the implementation of:
 *   - end(): to safely stop or release the motor hardware
 *   - isPinsSet(): to check if the required pins have been configured
 *
 * All specific motor driver classes (e.g., HBridge, ServoMotor) should inherit from Motor.
 */
class Motor {
  public:
    Motor() = default;
    /**
     * @brief Safely stop or release the motor hardware (pure virtual).
     */
    virtual void end() = 0;
    /**
     * @brief Check if the required pins have been configured (pure virtual).
     * @return true if pins are set, false otherwise
     */
    virtual bool isPinsSet() const = 0;
};