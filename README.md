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
Done on ros2 trixie cli only
1. Clone to `~/rover_ws/src`
2. install base ros2 jazzy and build tools: install build-essential git wget curl python3-pip python3-venv locales
3. update locales: sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
use rospian repo
install ros-jazzy-cv-bridge python3-serial
5. Run `colcon build --cmake-args -DBUILD_TESTING=OFF`
6. fix missing package errors if any
7. `source install/setup.bash`
8. `ros2 launch rover_control rover_bringup.launch.py`
