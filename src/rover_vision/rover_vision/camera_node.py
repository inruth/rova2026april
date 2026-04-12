#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Publisher for the brain (Uncompressed BGR8)
        self.pub_raw = self.create_publisher(Image, '/image_raw', 10)
        
        # Publisher for the laptop (Compressed JPEG to save Wi-Fi bandwidth)
        self.pub_comp = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        
        # Reduce FPS to ~10 to save Raspberry Pi CPU!
        self.timer = self.create_timer(0.1, self.capture_frame)
        
        # Start OpenCV Video Capture
        self.cap = cv2.VideoCapture(0)
        
        # Force a standard, low-processing resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.bridge = CvBridge()
        self.get_logger().info("Custom Camera Node Started at 10 FPS!")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame from camera.")
            return
        frame = cv2.flip(frame, -1)

        # 1. Publish Raw Image for color_tracker.py
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        raw_msg.header.frame_id = "camera_link"
        self.pub_raw.publish(raw_msg)

        # 2. Publish Compressed Image manually for rqt_image_view
        comp_msg = CompressedImage()
        comp_msg.header = raw_msg.header
        comp_msg.format = "jpeg"
        # OpenCV converts the frame directly to a jpeg byte array
        comp_msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
        self.pub_comp.publish(comp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

