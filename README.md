# Autonomous Soil Analysis Rover (ROS 2 Jazzy)

An autonomous platform designed for precision agriculture/remote exploration, integrating computer vision, sensor fusion, and soil health monitoring.

## System Architecture
* **Compute:** Raspberry Pi 4B (ROS 2 Jazzy)
* **Low-Level Control:** Arduino Nano (via Serial Bridge)
* **Navigation:** Extended Kalman Filter (EKF) fusing Encoder and IMU data via `robot_localization`.
* **Vision:** OpenCV-based line tracking for autonomous row navigation.
* **Actuation:** BTS7960 High-current drivers for linear soil-probing actuator.

## Core Packages
* `rover_control`: Hardware interface, Mother Node (behavior tree logic), and telemetry.
* `rover_vision`: Image processing and spatial detection.
* `nano_firmware`: C++ firmware for the Arduino hardware interface.

## Setup
1. Clone to `~/rover_ws/src`
2. install base ros2 jazzy and build tools
3. Run `colcon build --cmake-args -DBUILD_TESTING=OFF`
4. fix missing package errors if any
5. `source install/setup.bash`
6. `ros2 launch rover_control rover_bringup.launch.py`
