#!/usr/bin/env python3
"""Real-time robot position via TF (vodom -> base_link)."""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TfMonitor(Node):
    def __init__(self):
        super().__init__("tf_monitor")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.tick)  # 5Hz

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "vodom", "base_link", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            q = t.transform.rotation
            # quaternion -> yaw
            import math
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            self.get_logger().info(
                f"x={x:+.3f}  y={y:+.3f}  z={z:+.3f}  yaw={math.degrees(yaw):+.1f}°")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TfMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
