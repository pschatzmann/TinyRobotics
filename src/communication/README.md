# Communication Module

This module provides classes and utilities for handling communication protocols and data exchange in robotics applications. It is designed to support a wide range of communication needs, including both wired and wireless protocols commonly used in embedded and Arduino-based robotics.

## Communication Protocols

Here is an overview of the most relevant communication protocols that you can choose from: 

- **UDP**: Lightweight, connectionless network communication for fast data exchange.
- **TCP**: Reliable, connection-oriented network communication for robust data transfer.
- **ESPNow**: Low-power, peer-to-peer wireless communication protocol for ESP32/ESP8266 devices.
- **IR (Infrared)**: Communication using infrared transmitters and receivers (e.g., remote controls).
- **433 MHz RF Modules**: Simple wireless communication using low-cost RF transmitter/receiver pairs.
- **LoRa**: Long-range, low-power wireless communication for IoT and robotics (e.g., SX127x modules).
- **Serial**: Standard UART/USART serial communication (hardware or software serial ports).
- **IEEE802_15_4**: Low-power wireless communication standard for mesh and sensor networks (e.g., Zigbee, Thread, 6LoWPAN).
- **Ultrasound**: Distance measurement and simple signaling using ultrasonic transceivers (e.g., HC-SR04).

## Message Formats

This section describes the supported message formats and protocols for exchanging data between devices, modules, or systems in TinyRobotics.

- **TinyRobotics Messages**: A lean remote control format
- **JSON and XML messages**: TinyRobotics Messages can be serialized to JSON or XML and forwarded to any destination Stream
- **NMEA**: Standard protocol for GPS and marine devices (see NMEAParser).
- **Spektrum Satellite**: A well defined remote control protocol

## Features

- Unified interface for sending and receiving data across multiple protocols using Arduino Streams.
- Utilities for parsing, encoding, and validating protocol-specific messages.
- Extensible design for adding new communication methods.
- Integration with coordinate and sensor modules for distributed robotics.

## External Libraries

The following external libraries are supported or recommended for advanced communication features:

- [PulseWire](https://github.com/pschatzmann/PulseWire):
  - Library for pulse-based communication, useful for protocols like PPM, PWM, and custom pulse encoding/decoding.
- [SpektrumSatellite](https://github.com/pschatzmann/SpektrumSatellite):
  - Library for interfacing with Spektrum DSM2/DSMX satellite receivers, enabling wireless RC communication.
- [ArduinoMavlinkDrone](https://github.com/pschatzmann/ArduinoMavlinkDrone):
  - Library for MAVLink protocol support, enabling communication with ground stations, autopilots, and drones.
- [ESP32TransceiverIEEE802_15_4](https://github.com/pschatzmann/ESP32TransceiverIEEE802_15_4):
  - Library for IEEE 802.15.4 transceiver support on ESP32, enabling mesh and sensor network communication.
- [RadioHead](http://www.airspayce.com/mikem/arduino/RadioHead/):
  - Widely used library for wireless communication with a variety of RF modules (e.g., 433/868/915 MHz, LoRa, ASK, FSK, etc.).
- [Rosserial Arduino Library](http://wiki.ros.org/rosserial_arduino):
  - Library for ROS-compatible serial communication, enabling integration of Arduino devices with ROS-based systems.

These libraries can be integrated with TinyRobotics communication classes to extend protocol support for robotics and drone applications.
