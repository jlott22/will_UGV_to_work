# UGV Semantic Search Platform

This repository contains the software stack for an autonomous outdoor Unmanned Ground Vehicle (UGV) performing GPS-guided search operations using RTK GNSS, LiDAR, onboard perception, and decentralized task allocation.

The system is designed around a modular architecture:

- **Navigation** handles RTK GPS, heading estimation, waypoint following, LiDAR processing, and vehicle control. :contentReference[oaicite:0]{index=0}
- **Task Management** maintains the search map, probability distributions, task allocation logic, obstacle tracking, and mission planning. :contentReference[oaicite:1]{index=1}
- **Perception** provides object and clue detections that influence search behavior.
- **MQTT** is used for inter-process communication on the Jetson Nano.

Current development is focused on validating decentralized search algorithms on a single UGV before expanding to multi-robot operation.

## Hardware

- Jetson Nano
- u-blox ZED-F9R RTK rover
- u-blox ZED-F9P RTK base station
- RPLidar
- Arduino motor controller
- ESP32 communication modules
- RC vehicle platform

## Status

Active research and development project for autonomous search, mapping, and multi-robot coordination in outdoor environments.
