#!/usr/bin/env python3
"""
Ball color detector - subscribes to RGB camera, detects ball color via HSV masking.
Publishes result to /ball_color topic and prints to console.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# HSV ranges for common ball colors
COLOR_RANGES = {
    "red_low":   ([0,   120, 100], [10,  255, 255]),
    "red_high":  ([170, 120, 100], [180, 255, 255]),
    "orange":    ([10,  120, 100], [25,  255, 255]),
    "yellow":    ([25,  120, 100], [35,  255, 255]),
    "green":     ([35,  100, 100], [85,  255, 255]),
    "blue":      ([100, 120, 100], [130, 255, 255]),
    "purple":    ([130, 100, 100], [160, 255, 255]),
    "white":     ([0,   0,   200], [180, 30,  255]),
}

# Merge red ranges, map display names
DISPLAY_MAP = {
    "red_low": "red", "red_high": "red",
    "orange": "orange", "yellow": "yellow",
    "green": "green", "blue": "blue",
    "purple": "purple", "white": "white",
}


class BallColorDetector(Node):
    def __init__(self):
        super().__init__("ball_color_detector")
        self.bridge = CvBridge()
        self.declare_parameter("image_topic", "/rgb_camera/image_raw")
        image_topic = self.get_parameter("image_topic").value

        # Use sensor QoS for camera
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            Image, image_topic, self._image_cb, qos)
        self.pub = self.create_publisher(String, "/ball_color", 10)
        self.get_logger().info(f"[BallColor] Listening on {image_topic}")

    def _image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        h, w = cv_img.shape[:2]

        # Search in center region (where the ball is most likely)
        cx, cy = w // 2, h // 2
        roi_size = min(w, h) // 3
        roi = hsv[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

        best_color = None
        best_pixels = 0

        for name, (lo, hi) in COLOR_RANGES.items():
            mask = cv2.inRange(roi, np.array(lo), np.array(hi))
            count = cv2.countNonZero(mask)
            display = DISPLAY_MAP[name]
            if count > best_pixels:
                best_pixels = count
                best_color = display

        threshold = roi.shape[0] * roi.shape[1] * 0.01  # 1% of ROI
        if best_color and best_pixels > threshold:
            out = String()
            out.data = best_color
            self.pub.publish(out)
            self.get_logger().info(
                f"[BallColor] Detected: {best_color} ({best_pixels} px)")

            # Show annotated image (for debugging with display)
            cv2.putText(cv_img, best_color, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.rectangle(cv_img,
                          (cx - roi_size, cy - roi_size),
                          (cx + roi_size, cy + roi_size),
                          (0, 255, 0), 2)
        else:
            self.get_logger().info("[BallColor] No ball detected")

        # Save latest frame for debugging
        cv2.imwrite("/tmp/ball_frame.jpg", cv_img)


def main(args=None):
    rclpy.init(args=args)
    node = BallColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
