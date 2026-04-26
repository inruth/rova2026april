# Copyright 2026 Visruth K
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the EKF config file
    rover_control_dir = get_package_share_directory('rover_control')
    ekf_config_path = os.path.join(rover_control_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. The Arduino Serial Bridge
        Node(
            package='rover_control',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        ),

        # 2. Static Transform Publisher (base_link to imu_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', 
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
            ],
            output='screen'
        ),

        # 3. EKF Odometry/IMU Fusion Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 4. Custom Python Camera Publisher (Replaces v4l2_camera)
        Node(
            package='rover_vision', 
            executable='camera_node',
            name='camera_node',
            output='screen',
            prefix=['libcamerify']
        ),

        # 5. The Sensor Fusion / Autonomous Brain
        Node(
            package='rover_vision',
            executable='vision_processor',
            name='vision_processor',
            output='screen'
        ),

        # 6. The Mother Node (State Machine & Safety Firewall)
        Node(
            package='rover_control',
            executable='rover_behavior',
            name='rover_behavior',
            output='screen'
        ),

        # 7. NEW: Raspberry Pi Hardware Node (Actuator & Soil Sensors)
        Node(
            package='rover_control',
            executable='pi_hardware_node',
            name='pi_hardware_node',
            output='screen'
        )
    ])

