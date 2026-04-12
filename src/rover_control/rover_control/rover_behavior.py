#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Range
import math
import time

class RoverBehavior(Node):
    def __init__(self):
        super().__init__('rover_behavior')

        self.STATE_FOLLOWING = 0
        self.STATE_IMU_RADAR = 1       
        self.STATE_GREEN_ALIGN = 2      
        self.STATE_EXTRACTING = 3
        self.STATE_RECOVERY = 4
        self.STATE_MANUAL_OVERRIDE = 5
        self.STATE_MICRO_SEARCH = 6

        self.current_state = self.STATE_FOLLOWING

        self.steer_sub = self.create_subscription(Twist, '/vision/steering', self.steer_callback, 10)
        self.status_sub = self.create_subscription(String, '/vision/status', self.status_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, '/vision/green_trigger', self.trigger_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sonic_sub = self.create_subscription(Range, '/ultrasonic/range', self.sonic_callback, 10)
        self.teleop_sub = self.create_subscription(Twist, '/teleop_cmd', self.teleop_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_vision_cmd = Twist()
        self.current_vision_status = "BLIND"
        self.current_teleop_cmd = Twist()

        # --- NEW: Heartbeat tracker for the Deadman Switch ---
        self.last_vision_heartbeat = time.time()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0          

        self.radar_center_yaw = 0.0
        self.radar_stage = 0
        self.radar_sweeps_completed = 0
        self.radar_pulse_on = False
        self.last_radar_time = time.time()
        self.radar_spin_dur = 0.20  
        self.radar_look_dur = 0.50
        self.micro_step = 0
        self.micro_timer = 0.0
        self.brake_dir = 0.0

        self.align_start_x = 0.0
        self.align_start_y = 0.0
        self.blind_drive_distance = 0.235 
        self.align_speed = 0.35 
        self.obstacle_distance = float('inf')
        self.safe_stop_distance = 0.15 

        self.extraction_start_time = 0.0
        self.extraction_duration = 30.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Mother Node V18 Started. Deadman Switch Active.")

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def steer_callback(self, msg):
        self.current_vision_cmd = msg
        self.last_vision_heartbeat = time.time() # Reset heartbeat!

    def status_callback(self, msg):
        self.current_vision_status = msg.data
        self.last_vision_heartbeat = time.time() # Reset heartbeat!

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def sonic_callback(self, msg):
        self.obstacle_distance = msg.range

    def teleop_callback(self, msg):
        if self.current_state != self.STATE_MANUAL_OVERRIDE:
            self.get_logger().warn("MANUAL OVERRIDE ENGAGED! Autonomous routines terminated.")
            self.current_state = self.STATE_MANUAL_OVERRIDE
        self.current_teleop_cmd = msg

    def trigger_callback(self, msg):
        if msg.data == True and self.current_state == self.STATE_FOLLOWING:
            self.align_start_x = self.current_x
            self.align_start_y = self.current_y
            self.current_state = self.STATE_GREEN_ALIGN

    def get_angle_difference(self, target, current):
        diff = target - current
        return math.atan2(math.sin(diff), math.cos(diff))

    def control_loop(self):
        cmd = Twist()

        if self.current_state == self.STATE_FOLLOWING:
            # Check if the word "BLIND" is anywhere in the status (e.g., "BLIND_LEFT")
            if "BLIND" in self.current_vision_status:
                self.get_logger().warn(f"Camera Blind! Priority Radar ({self.current_vision_status}).")
                self.current_state = self.STATE_IMU_RADAR
                self.radar_center_yaw = self.current_yaw
                self.radar_sweeps_completed = 0  # Reset the fail counter

                # --- PRIORITY SWEEP DIRECTION ---
                if "RIGHT" in self.current_vision_status:
                    self.radar_stage = 2 # Start by sweeping Right
                else:
                    self.radar_stage = 1 # Start by sweeping Left
            else:
                cmd.linear.x = self.current_vision_cmd.linear.x
                cmd.angular.z = self.current_vision_cmd.angular.z

        elif self.current_state == self.STATE_IMU_RADAR:
            if self.current_vision_status == "TRACKING":
                self.get_logger().info("IMU Radar detected tape! Engaging Micro-Search Wiggle.")
                self.current_state = self.STATE_MICRO_SEARCH
                self.micro_step = 0
                self.micro_timer = time.time()
                # If sweeping Left (1), brake Right (-1.0).
                self.brake_dir = -1.0 if self.radar_stage == 1 else 1.0 
            else:
                yaw_drift = self.get_angle_difference(self.current_yaw, self.radar_center_yaw)

                # --- BIDIRECTIONAL BOUNCE LOGIC (With shorter 0.8 rad sweep) ---
                if self.radar_stage == 1 and yaw_drift > 0.8:
                    self.get_logger().info("Left sweep maxed out. Bouncing Right...")
                    self.radar_stage = 2
                    self.radar_sweeps_completed += 1
                elif self.radar_stage == 2 and yaw_drift < -0.8:
                    self.get_logger().info("Right sweep maxed out. Bouncing Left...")
                    self.radar_stage = 1
                    self.radar_sweeps_completed += 1

                # If we bounced back and forth twice and still missed, give up.
                if self.radar_sweeps_completed >= 2:
                    self.get_logger().error("RADAR FAILED. Tight arc cleared in both directions. Halting.")
                    self.current_state = self.STATE_MANUAL_OVERRIDE 

                current_time = time.time()
                if self.radar_pulse_on:
                    if (current_time - self.last_radar_time) > self.radar_spin_dur:
                        self.radar_pulse_on = False
                        self.last_radar_time = current_time
                else:
                    if (current_time - self.last_radar_time) > self.radar_look_dur:
                        self.radar_pulse_on = True
                        self.last_radar_time = current_time

                if self.radar_pulse_on:
                    cmd.angular.z = 0.6 if self.radar_stage == 1 else -0.6
                else:
                    cmd.angular.z = 0.0
                    
        elif self.current_state == self.STATE_MICRO_SEARCH:
            # INSTANT ABORT: If the camera sees the tape at any point during this dance, we win!
            if self.current_vision_status == "TRACKING" and self.micro_step > 0:
                self.get_logger().info("Micro-Search locked on! Handing off to Vision Node.")
                self.current_state = self.STATE_FOLLOWING
                cmd.angular.z = 0.0
            else:
                elapsed = time.time() - self.micro_timer

                # Step 0: The Initial Hard Brake (150ms reverse pulse)
                if self.micro_step == 0:
                    if elapsed < 0.15:
                        cmd.angular.z = 0.8 * self.brake_dir
                    else:
                        self.micro_step = 1
                        self.micro_timer = time.time()
                        cmd.angular.z = 0.0

                # Step 1: Look Pause (500ms)
                elif self.micro_step == 1:
                    if elapsed > 0.50:
                        self.micro_step = 2
                        self.micro_timer = time.time()

                # Step 2: Micro-Pulse Opposite (100ms)
                elif self.micro_step == 2:
                    if elapsed < 0.10:
                        cmd.angular.z = 0.6 * self.brake_dir
                    else:
                        self.micro_step = 3
                        self.micro_timer = time.time()
                        cmd.angular.z = 0.0

                # Step 3: Look Pause (500ms)
                elif self.micro_step == 3:
                    if elapsed > 0.50:
                        self.micro_step = 4
                        self.micro_timer = time.time()

                # Step 4: Micro-Pulse Original Direction (100ms wiggle back)
                elif self.micro_step == 4:
                    if elapsed < 0.10:
                        cmd.angular.z = 0.6 * (-self.brake_dir) # Reverse the brake direction!
                    else:
                        self.micro_step = 5
                        self.micro_timer = time.time()
                        cmd.angular.z = 0.0

                # Step 5: Final Look Pause (500ms)
                elif self.micro_step == 5:
                    if elapsed > 0.50:
                        self.get_logger().warn("Micro-Search failed. Falling back to Main Radar.")
                        self.current_state = self.STATE_IMU_RADAR
                        self.radar_center_yaw = self.current_yaw # Re-center the sweep
                        self.radar_sweeps_completed = 0

        elif self.current_state == self.STATE_GREEN_ALIGN:
            dx = self.current_x - self.align_start_x
            dy = self.current_y - self.align_start_y
            distance_driven = math.sqrt(dx**2 + dy**2)

            if distance_driven < self.blind_drive_distance:
                cmd.linear.x = self.align_speed
            else:
                cmd.linear.x = 0.0
                self.extraction_start_time = time.time()
                self.current_state = self.STATE_EXTRACTING

        elif self.current_state == self.STATE_EXTRACTING:
            cmd.linear.x = 0.0
            elapsed = time.time() - self.extraction_start_time
            if elapsed > self.extraction_duration:
                self.align_start_x = self.current_x
                self.align_start_y = self.current_y
                self.current_state = self.STATE_RECOVERY

        elif self.current_state == self.STATE_RECOVERY:
            dx = self.current_x - self.align_start_x
            dy = self.current_y - self.align_start_y
            distance_driven = math.sqrt(dx**2 + dy**2)

            if distance_driven < 0.10:
                cmd.linear.x = self.align_speed
            else:
                self.current_state = self.STATE_FOLLOWING

        elif self.current_state == self.STATE_MANUAL_OVERRIDE:
            cmd.linear.x = self.current_teleop_cmd.linear.x
            cmd.angular.z = self.current_teleop_cmd.angular.z

        # --- MASTER SAFETY FIREWALL ---
        if self.obstacle_distance <= self.safe_stop_distance and cmd.linear.x > 0:
            cmd.linear.x = 0.0
            self.get_logger().warn(f"COLLISION AVOIDANCE! Obstacle detected at {self.obstacle_distance:.2f}m")

        # --- NEW: DEADMAN SWITCH ---
        # If we haven't received a vision command in 0.5 seconds, assume the camera node crashed.
        # Cut power immediately to prevent a zombie runaway rover.
        if (time.time() - self.last_vision_heartbeat) > 0.5 and self.current_state == self.STATE_FOLLOWING:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().error("DEADMAN SWITCH TRIGGERED! Vision Node unresponsive. Halting motors.")

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RoverBehavior()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







