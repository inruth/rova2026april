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
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.steer_pub = self.create_publisher(Twist, '/vision/steering', 10)
        self.status_pub = self.create_publisher(String, '/vision/status', 10)
        self.green_pub = self.create_publisher(Bool, '/vision/green_trigger', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/debug_view', 1)
        self.black_mask_pub = self.create_publisher(Image, '/vision/black_mask', 1)
        self.green_mask_pub = self.create_publisher(Image, '/vision/green_mask', 1)
        self.bridge = CvBridge()

        self.base_speed = 0.55      
        self.kp_steer = 0.005      
        
        self.blind_frames = 0       
        self.recovery_frames = 0
        self.last_seen_side = "LEFT"

        # ---Forward Stutter-Step Variables ---
        self.track_pulse_on = False
        self.last_track_time = time.time()
        self.track_drive_dur = 0.15  # 150ms of forward gas
        self.track_look_dur = 0.35   # 350ms of brakes to take a clear photo
        self.node_start_time = time.time()
        self.warmup_dur = 3.0

        self.get_logger().info("Vision Processor V19: Settle Pause + Forward Stutter Active.")
    #
    def get_contour_data(self, mask_slice):
        contours, _ = cv2.findContours(mask_slice, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            
            if cv2.contourArea(largest) > 1500 and w > 30:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return cx, cy, w, h
        return None, None, None, None

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        height, width, _ = frame.shape
        roi = frame[int(height/2):height, 0:width]
        roi_h, roi_w = roi.shape[:2]
        debug_img = roi.copy()

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, black_mask = cv2.threshold(blur, 30, 255, cv2.THRESH_BINARY_INV)

        split_y = int(roi_h / 2)
        top_mask = black_mask[0:split_y, 0:roi_w]
        bot_mask = black_mask[split_y:roi_h, 0:roi_w]

        top_cx, top_cy, top_w, top_h = self.get_contour_data(top_mask)
        bot_cx, bot_cy, bot_w, bot_h = self.get_contour_data(bot_mask)

        steer_cmd = Twist()
        status_msg = String()
        cv2.line(debug_img, (0, split_y), (roi_w, split_y), (255, 0, 0), 2)

        # ---CAMERA WARMUP BLOCK ---
        # Keep Mother Node pacified and satisfy the Deadman Switch while the lens opens
        if (time.time() - self.node_start_time) < self.warmup_dur:
            status_msg.data = "TRACKING" 
            steer_cmd.linear.x = 0.0
            steer_cmd.angular.z = 0.0
            cv2.putText(debug_img, "SYSTEM WARMUP...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            self.steer_pub.publish(steer_cmd)
            self.status_pub.publish(status_msg)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
            return # Exit the callback early so it doesn't process the black frames!

        # --- Settle Pause followed by Forward Stutter-Step ---
        if top_cx is not None:
            # Saves "RIGHT" or "LEFT" based on the tape's X-coordinate
            self.last_seen_side = "RIGHT" if top_cx > (roi_w / 2) else "LEFT"
            
            # 1. Trigger the settle pause if just coming out of a blind radar sweep
            if self.blind_frames > 0:
                self.recovery_frames = 10 # 10 frames = 1 second pause
                self.blind_frames = 0
                
            status_msg.data = "TRACKING" 
            
            # 2. Execute Settle Pause to let Active Brake momentum die completely
            if self.recovery_frames > 0:
                steer_cmd.linear.x = 0.0
                steer_cmd.angular.z = 0.0
                self.recovery_frames -= 1
                cv2.putText(debug_img, f"SETTLING MOMENTUM... ({self.recovery_frames})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.circle(debug_img, (top_cx, top_cy), 10, (0, 255, 255), -1) 
            
            # 3. Momentum is gone. Execute Forward Stutter-Step!
            else:
                error = (roi_w / 2) - top_cx
                
                current_time = time.time()
                if self.track_pulse_on:
                    if (current_time - self.last_track_time) > self.track_drive_dur:
                        self.track_pulse_on = False
                        self.last_track_time = current_time
                else:
                    if (current_time - self.last_track_time) > self.track_look_dur:
                        self.track_pulse_on = True
                        self.last_track_time = current_time
                        
                if self.track_pulse_on:
                    steer_cmd.linear.x = self.base_speed
                    steer_cmd.angular.z = float(self.kp_steer * error)
                    cv2.putText(debug_img, "TRACKING - DRIVE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    steer_cmd.linear.x = 0.0
                    steer_cmd.angular.z = 0.0
                    cv2.putText(debug_img, "TRACKING - LOOK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                cv2.circle(debug_img, (top_cx, top_cy), 10, (0, 255, 0), -1)
                
        elif bot_cx is not None:
            # --- Update Directional Memory during Approach ---
            self.last_seen_side = "RIGHT" if bot_cx > (roi_w / 2) else "LEFT"
            
            self.blind_frames = 0
            self.recovery_frames = 0
            status_msg.data = "TRACKING" 
            error = (roi_w / 2) - bot_cx
            
            # Apply the same stutter-step logic to the approach phase
            current_time = time.time()
            if self.track_pulse_on:
                if (current_time - self.last_track_time) > self.track_drive_dur:
                    self.track_pulse_on = False
                    self.last_track_time = current_time
            else:
                if (current_time - self.last_track_time) > self.track_look_dur:
                    self.track_pulse_on = True
                    self.last_track_time = current_time
                    
            if self.track_pulse_on:
                steer_cmd.linear.x = self.base_speed * 0.7 
                steer_cmd.angular.z = float(self.kp_steer * error * 1.5) 
                cv2.putText(debug_img, "APPROACHING (DRIVE)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                steer_cmd.linear.x = 0.0
                steer_cmd.angular.z = 0.0
                cv2.putText(debug_img, "APPROACHING (LOOK)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
            cv2.circle(debug_img, (bot_cx, bot_cy + split_y), 10, (0, 165, 255), -1) 

        else:
            self.recovery_frames = 0
            self.blind_frames += 1
            if self.blind_frames < 6: 
                status_msg.data = "TRACKING"
                steer_cmd.linear.x = 0.0
                steer_cmd.angular.z = 0.0 
                cv2.putText(debug_img, "BLUR - BRAKING...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                status_msg.data = f"BLIND_{self.last_seen_side}"
                steer_cmd.linear.x = 0.0
                steer_cmd.angular.z = 0.0
                cv2.putText(debug_img, f"BLIND - RADAR {self.last_seen_side}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.steer_pub.publish(steer_cmd)
        self.status_pub.publish(status_msg)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        green_lower = np.array([40, 100, 100])
        green_upper = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_triggered = Bool()
        green_triggered.data = False

        if len(green_contours) > 0:
            c_green = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(c_green) > 1000:
                x, y, w, h = cv2.boundingRect(c_green)
                if w > (h * 0.5) and h < 60: 
                    bottom_y_of_green = y + h
                    if bottom_y_of_green >= (roi_h - 20):
                        green_triggered.data = True
                cv2.rectangle(debug_img, (x, y), (x+w, y+h), (255, 0, 255), 2)

        self.green_pub.publish(green_triggered)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
        self.black_mask_pub.publish(self.bridge.cv2_to_imgmsg(black_mask, encoding="mono8"))
        self.green_mask_pub.publish(self.bridge.cv2_to_imgmsg(green_mask, encoding="mono8"))

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



