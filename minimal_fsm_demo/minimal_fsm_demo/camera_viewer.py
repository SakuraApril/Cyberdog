#!/usr/bin/env python3
"""
Simple camera viewer for Cyberdog simulation.
Shows what the robot camera is seeing in a real-time OpenCV window.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self.bridge = CvBridge()
        self._window = "Cyberdog Camera View"
        self._frame_seen = False

        self.declare_parameter("image_topic", "/rgb_camera/image_raw")
        topic = self.get_parameter("image_topic").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, topic, self.on_image, qos)
        self.get_logger().info(f"[Viewer] Subscribed to: {topic}")
        self.get_logger().info("[Viewer] Press 'q' in image window to quit")
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 960, 540)
        self._show_placeholder()

    def _show_placeholder(self):
        img = np.zeros((540, 960, 3), dtype=np.uint8)
        cv2.putText(img, "Waiting for camera...", (40, 280),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imshow(self._window, img)
        cv2.waitKey(1)

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        self._frame_seen = True
        cv2.imshow(self._window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("[Viewer] Quit requested")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
