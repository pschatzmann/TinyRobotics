# TinyRobotics Utils Module

This directory contains utility classes and helpers that support the core robotics, mapping, and planning functionality of the TinyRobotics library. These utilities are designed for embedded and desktop use, with a focus on efficiency and minimal dependencies.

## Contents

- **LoggerClass.h**  
  Lightweight logging utility with support for log levels (info, warning, error) and output to serial or other streams.

- **Config.h**  
  Configuration and parameter management for compile-time and run-time settings.

- **Common.h**  
  Common macros, type definitions, and utility functions used throughout the library.

- **VectorFromArray.h**  
  Fixed-capacity vector implementation using `std::array` for environments where dynamic allocation is undesirable.

- **Utils.h**  
  General-purpose utility functions and helpers.

## Class Documentation

- [utilities](https://pschatzmann.github.io/TinyRobotics/group__utils.html)



## Typical Usage

- Logging messages and errors during development or runtime.
- Managing configuration parameters for your robot or application.
- Using fixed-size containers and helpers in memory-constrained environments.

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [planning/](../planning/) for path planning and navigation
- [maps/](../maps/) for map and path data structures

---

This module is designed to provide robust, efficient utilities for embedded and desktop robotics applications.