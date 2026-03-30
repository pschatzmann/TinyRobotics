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


## See Examples

- [RC Vehicle Control](../../../examples/vehicles/RCVehicle/RCVehicle.ino)
- [IMU 2D Example](../../../examples/imu/imu2d/imu2d.ino)
- [2D Frame Example](../../../examples/coordinates&frames/frame2d/frame2d.ino)
- [3D Frame Example](../../../examples/coordinates&frames/frame3d/frame3d.ino)
- [Record GPS Example](../../../examples/coordinates&frames/record-gps/record-gps.ino)
- [Gridmap A*](../../../examples/maps&planning/gridmap-astar/girdmap-astar.ino)
- [Gridmap Dijkstra](../../../examples/maps&planning/gridmap-dijkstra/girdmap-dijkstra.ino)
- [Pathmap A*](../../../examples/maps&planning/pathmap-astar/pathmap-astar.ino)
- [Planned Path](../../../examples/maps&planning/plannedpath/plannedpath.ino)
- [Pointcloud A*](../../../examples/maps&planning/pointcloud-astar/pointcloud-astar.ino)
- [Wheel Encoder](../../../examples/sensors/WheelEncoder/WheelEncoder.ino)
- [Range Sensor](../../../examples/sensors/RangeSensor/RangeSensor.ino)
- [Camera Line Follower](../../../examples/sensors/CameraLineFollower/CameraLineFollower.ino)
