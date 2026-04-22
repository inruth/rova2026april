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
2. Run `colcon build --cmake-args -DBUILD_TESTING=OFF`
3. `source install/setup.bash`
4. `ros2 launch rover_control rover_bringup.launch.py`

## License
Proprietary - All Rights Reserved.

(This project is Proprietary. Personal use for learning and hobbyist experimentation is encouraged. See LICENSE.txt for full terms.)

```mermaid
graph LR
    %% Defining Styles
    classDef hardware fill:#f9f,stroke:#333,stroke-width:2px;
    classDef node fill:#bbf,stroke:#333,stroke-width:2px;
    classDef topic fill:#dfd,stroke:#333,stroke-width:1px,stroke-dasharray: 5 5;

    %% External Hardware Layer
    subgraph External_Hardware
        CAM[Raspberry Pi Camera]:::hardware
        NANO[Arduino Nano]:::hardware
        GPIO[Pi GPIO & 1-Wire]:::hardware
    end

    %% Vision Package
    subgraph rover_vision Package
        CAM_NODE(v4l2_camera_node):::node
        VP(vision_processor):::node
    end

    %% Control Package
    subgraph rover_control Package
        SB(serial_bridge):::node
        RB(rover_behavior):::node
        HW(pi_hardware_node):::node
        EKF(robot_localization):::node
        TELEOP(teleop_keyboard):::node
    end

    %% Data Flow / Connections
    CAM -->|Video Stream| CAM_NODE
    CAM_NODE -->|/image_raw| VP
    
    VP -->|/vision/steering| RB
    VP -->|/vision/status| RB
    VP -->|/vision/green_trigger| RB

    NANO <-->|Serial UART| SB
    
    SB -->|/odom| RB
    SB -->|/odom| EKF
    SB -->|/imu/data_raw| EKF
    SB -->|/ultrasonic/range| RB

    TELEOP -->|/teleop_cmd| RB
    
    RB -->|/cmd_vel| SB
    RB -->|/cmd_vel| HW

    SB -->|Motor PWMs| NANO
    HW -->|Actuator / Relays| GPIO
```
