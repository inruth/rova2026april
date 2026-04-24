#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
import serial
import math
from tf2_ros import TransformBroadcaster

class RoverSerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Wheel properties
        self.tpm_fl = 67.66
        self.tpm_fr = 46.00
        self.tpm_rl = 65.33
        self.tpm_rr = 55.33
        
        self.track_width = 0.1905  
        self.max_pwm = 255.0            
        self.max_speed_mps = 1.0        

        # Serial Setup
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Nano on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Nano: {e}")
            raise SystemExit

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_fl = 0
        self.last_fr = 0
        self.last_rl = 0
        self.last_rr = 0
        self.last_time = self.get_clock().now()

        # --- ROS 2 PUBLISHERS & SUBSCRIBERS ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10) 
        
        # <-- Added Ultrasonic Publisher
        self.ultrasonic_pub = self.create_publisher(Range, 'ultrasonic/range', 10) 
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Fast 100Hz timer to prevent serial buffer buildup
        self.timer = self.create_timer(0.01, self.read_serial_and_publish)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.track_width / 2.0)
        v_right = v + (w * self.track_width / 2.0)

        pwm_left = int((v_left / self.max_speed_mps) * self.max_pwm)
        pwm_right = int((v_right / self.max_speed_mps) * self.max_pwm)

        # --- NEW: DEADBAND COMPENSATION ---
        # 85 provides the minimum torque to break static friction on tile.
        deadband = 85 

        if pwm_left > 0 and pwm_left < deadband: pwm_left = deadband
        if pwm_left < 0 and pwm_left > -deadband: pwm_left = -deadband
        
        if pwm_right > 0 and pwm_right < deadband: pwm_right = deadband
        if pwm_right < 0 and pwm_right > -deadband: pwm_right = -deadband

        # Hard caps
        pwm_left = max(min(pwm_left, 255), -255)
        pwm_right = max(min(pwm_right, 255), -255)

        command = f"M:{pwm_left},{pwm_right}\n"
        self.arduino.write(command.encode('utf-8'))

    def read_serial_and_publish(self):
        # Read ALL waiting lines in the buffer to clear the backlog
        while self.arduino.in_waiting:
            try:
                line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                
                # Parse Ticks
                if line.startswith("T:"):
                    parts = line[2:].split(',')
                    if len(parts) == 4:
                        try:
                            current_fl, current_fr, current_rl, current_rr = map(int, parts)
                            self.update_odometry(current_fl, current_fr, current_rl, current_rr)
                        except ValueError:
                            pass # Silently drop corrupted string
                
                # Parse IMU
                elif line.startswith("I:"):
                    parts = line[2:].split(',')
                    if len(parts) == 6:
                        try:
                            ax, ay, az, gx, gy, gz = map(float, parts)
                            
                            # --- DEADBAND FILTER ---
                            if abs(gx) < 0.05: gx = 0.0
                            if abs(gy) < 0.05: gy = 0.0
                            if abs(gz) < 0.05: gz = 0.0
                            
                            self.publish_imu(ax, ay, az, gx, gy, gz)
                        except ValueError:
                            pass

                # --- Parse Ultrasonic with Noise Filter ---
                elif line.startswith("U:"):
                    try:
                        dist_str = line[2:].strip()
                        
                        range_msg = Range()
                        range_msg.header.stamp = self.get_clock().now().to_msg()
                        range_msg.header.frame_id = "ultrasonic_link"
                        
                        # Hardware specs for HC-SR04
                        range_msg.radiation_type = Range.ULTRASOUND
                        range_msg.field_of_view = 0.26
                        range_msg.min_range = 0.02 
                        range_msg.max_range = 2.50   
                        
                        if dist_str == "MAX":
                            range_msg.range = float('inf')
                        else:
                            # Convert cm from Arduino to meters for ROS 2
                            raw_dist = float(dist_str) / 100.0 
                            
                            # --- NOISE FILTER ---
                            # Ignore false positives under 4cm (hardware blindspot/floor echoes)
                            if raw_dist < 0.04:
                                range_msg.range = float('inf')
                            else:
                                range_msg.range = raw_dist
                                
                        self.ultrasonic_pub.publish(range_msg)
                    except ValueError:
                        pass # Silently drop corrupted string
                            
            except Exception as e:
                pass # Prevent node from crashing on bad serial read                        

    def publish_imu(self, ax, ay, az, gx, gy, gz):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Linear Acceleration (m/s^2)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        # Angular Velocity (rad/s)
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # --- EKF COVARIANCE FIX ---
        imu_msg.orientation_covariance[0] = -1.0 
        
        # Give a small variance to angular velocity so the EKF trusts it
        imu_msg.angular_velocity_covariance[0] = 0.01  # X variance
        imu_msg.angular_velocity_covariance[4] = 0.01  # Y variance
        imu_msg.angular_velocity_covariance[8] = 0.01  # Z variance (Yaw)

        # Give a small variance to linear acceleration 
        imu_msg.linear_acceleration_covariance[0] = 0.04
        imu_msg.linear_acceleration_covariance[4] = 0.04
        imu_msg.linear_acceleration_covariance[8] = 0.04
        
        self.imu_pub.publish(imu_msg)

    def update_odometry(self, fl, fr, rl, rr):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        delta_fl = fl - self.last_fl
        delta_fr = fr - self.last_fr
        delta_rl = rl - self.last_rl
        delta_rr = rr - self.last_rr

        d_fl = delta_fl / self.tpm_fl
        d_fr = delta_fr / self.tpm_fr
        d_rl = delta_rl / self.tpm_rl
        d_rr = delta_rr / self.tpm_rr

        d_left = (d_fl + d_rl) / 2.0
        d_right = (d_fr + d_rr) / 2.0

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.track_width

        if d_center != 0 or d_theta != 0:
            self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
            self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
            self.theta += d_theta

        vx = d_center / dt if dt > 0 else 0.0
        vth = d_theta / dt if dt > 0 else 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        # robot_localization will broadcast the corrected odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        # self.tf_broadcaster.sendTransform(t) # <-- DISABLED FOR EKF

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        self.last_fl, self.last_fr = fl, fr
        self.last_rl, self.last_rr = rl, rr
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = RoverSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arduino.write(b"M:0,0\n") 
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




