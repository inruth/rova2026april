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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature
from gpiozero import DigitalOutputDevice, DigitalInputDevice
import glob
import time
import os

class PiHardwareNode(Node):
    def __init__(self):
        super().__init__('pi_hardware_node')

        # --- GPIO SETUP ---
        # Using pins identified in your working setup
        self.r_en = DigitalOutputDevice(22)
        self.l_en = DigitalOutputDevice(23)
        self.rpwm = DigitalOutputDevice(12)
        self.lpwm = DigitalOutputDevice(13)
        self.moisture = DigitalInputDevice(27)

        # Enable the motor driver immediately
        self.r_en.on()
        self.l_en.on()

        # Ensure actuator is stopped initially
        self.stop_actuator()

        # --- DS18B20 SETUP ---
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')
        self.base_dir = '/sys/bus/w1/devices/'
        try:
            self.device_folder = glob.glob(self.base_dir + '28*')[0]
            self.device_file = self.device_folder + '/w1_slave'
            self.temp_sensor_found = True
            self.get_logger().info("DS18B20 Temperature sensor initialized.")
        except IndexError:
            self.get_logger().error("DS18B20 sensor not found on 1-Wire bus!")
            self.temp_sensor_found = False

        # --- ROS 2 PUBLISHERS & SUBSCRIBERS ---
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.temp_pub = self.create_publisher(Temperature, 'soil/temperature', 10)
        self.moisture_pub = self.create_publisher(Bool, 'soil/moisture_alert', 10)

        self.sensor_timer = self.create_timer(1.0, self.publish_sensors)
        self.get_logger().info("Pi Hardware Node Started. Actuator ready.")

    def stop_actuator(self):
        self.rpwm.off()
        self.lpwm.off()

    def cmd_vel_callback(self, msg):
        
        z_cmd = msg.linear.z
        y_cmd = msg.linear.y 
        
        # Combine them so either input works
        combined_cmd = z_cmd + y_cmd

        if combined_cmd > 0.1:
            self.get_logger().info(f"Actuator Moving UP (cmd: {combined_cmd:.2f})")
            self.rpwm.on()
            self.lpwm.off()
        elif combined_cmd < -0.1:
            self.get_logger().info(f"Actuator Moving DOWN (cmd: {combined_cmd:.2f})")
            self.rpwm.off()
            self.lpwm.on()
        else:
            self.stop_actuator()

    def read_temp_raw(self):
        try:
            with open(self.device_file, 'r') as f:
                lines = f.readlines()
            return lines
        except Exception as e:
            return []

    def read_temp(self):
        if not self.temp_sensor_found:
            return 0.0
        
        lines = self.read_temp_raw()
        if not lines:
            return 0.0

        # Wait for valid reading from 1-Wire bus
        attempts = 0
        while lines[0].strip()[-3:] != 'YES' and attempts < 5:
            time.sleep(0.1)
            lines = self.read_temp_raw()
            attempts += 1
            
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
        return 0.0

    def publish_sensors(self):
        # Publish Temperature
        if self.temp_sensor_found:
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = 'actuator_link'
            temp_msg.temperature = self.read_temp()
            self.temp_pub.publish(temp_msg)

        # Publish Moisture
        moisture_msg = Bool()
        moisture_msg.data = not bool(self.moisture.value) 
        self.moisture_pub.publish(moisture_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down hardware node...")
        self.stop_actuator()
        self.r_en.close()
        self.l_en.close()
        self.rpwm.close()
        self.lpwm.close()
        self.moisture.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
