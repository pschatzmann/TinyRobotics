# TinyRobotics Utils Module

This directory contains utility classes and helpers that support the core robotics, mapping, and planning functionality of the TinyRobotics library. These utilities are designed for embedded and desktop use, with a focus on efficiency and minimal dependencies.

## Contents

- **LoggerClass.h**  
  Lightweight logging utility with support for log levels (info, warning, error) and output to serial or other streams.

- **Scheduler.h**  
  Simple cooperative scheduler for periodic or delayed task execution, suitable for microcontrollers and real-time loops.

- **Config.h**  
  Configuration and parameter management for compile-time and run-time settings.

- **Common.h**  
  Common macros, type definitions, and utility functions used throughout the library.

- **VectorFromArray.h**  
  Fixed-capacity vector implementation using `std::array` for environments where dynamic allocation is undesirable.

- **KalmanFilter.h**  
  Basic Kalman filter implementation for sensor fusion and state estimation.

- **Utils.h**  
  General-purpose utility functions and helpers.

## Typical Usage

- Logging messages and errors during development or runtime.
- Scheduling periodic sensor reads, control updates, or other tasks.
- Managing configuration parameters for your robot or application.
- Using fixed-size containers and helpers in memory-constrained environments.

## Example

```cpp
#include "utils/LoggerClass.h"
#include "utils/Scheduler.h"

TRLogger.info("Robot started!");

Scheduler scheduler;
scheduler.every(1000, []() {
    TRLogger.info("1 second passed");
});
while(true) scheduler.run();
```

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [planning/](../planning/) for path planning and navigation
- [maps/](../maps/) for map and path data structures

---

This module is designed to provide robust, efficient utilities for embedded and desktop robotics applications.