# Control Module Overview

This module provides essential classes for implementing control algorithms and state estimation in robotics and embedded systems. Below is an overview of the available classes:

## Classes

- **KalmanFilter**
	- Implements a simple Kalman Filter for state estimation. Useful for sensor fusion and filtering noisy measurements.

- **MovingAverage**
	- A template class for calculating the moving average of a sequence of values. Helps smooth out short-term fluctuations and highlight longer-term trends.

- **PIDController**
	- Provides a Proportional-Integral-Derivative (PID) controller for closed-loop control systems. Widely used for motor, position, and process control.

- **Scheduler**
	- A simple scheduler class that allows you to schedule and execute callbacks at specified intervals. Useful for periodic tasks and cooperative multitasking.

- **MotionController2D**
	- A 2D motion controller for path following and vehicle control. Integrates PID control for both speed (throttle) and steering, supports smooth acceleration/deceleration, and uses IMU feedback. Features configurable target accuracy for waypoint following and records the start coordinate for odometry or logging.

---

For detailed usage and API documentation, refer to the header files or the main project documentation.
