#!/usr/bin/env python3
"""
Stage 2 draft:
1) Turn 180 deg to scan one row of balls.
2) If orange is detected, move forward to hit it.
3) Continue scanning the next row.
"""

import enum
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

try:
    import lcm
except Exception:
    lcm = None

try:
    from minimal_fsm_demo.lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
except Exception:
    robot_control_cmd_lcmt = None


LCM_CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
CMD_CHANNEL = "robot_control_cmd"

MODE_PURE_DAMPER = 7
MODE_LOCOMOTION = 11
MODE_RECOVERY_STAND = 12
GAIT_STAND = 1
GAIT_TROT = 26
GAIT_TROT_SLOW = 27
CONTACT_ALL = 15


class State(enum.Enum):
    INIT = "INIT"
    STAND_UP = "STAND_UP"
    SCAN_TURN = "SCAN_TURN"
    GO_TO_ORANGE = "GO_TO_ORANGE"
    SHIFT_NEXT_ROW = "SHIFT_NEXT_ROW"
    DONE = "DONE"


class RobotCtrl:
    def __init__(self, node):
        self._node = node
        self._lcm_cmd = None
        self._lock = threading.Lock()
        self._cmd_msg = robot_control_cmd_lcmt()
        self._running = True
        self._life_count = 0

        if lcm is None or robot_control_cmd_lcmt is None:
            self._node.get_logger().error("LCM unavailable!")
            return

        self._lcm_cmd = lcm.LCM(LCM_CMD_URL)
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def send_cmd(self, mode, gait_id=0, vel_des=None, duration=0, step_height=None, contact=CONTACT_ALL):
        with self._lock:
            self._life_count = (self._life_count + 1) % 128
            msg = robot_control_cmd_lcmt()
            msg.mode = mode
            msg.gait_id = gait_id
            msg.contact = contact
            msg.duration = duration
            msg.life_count = self._life_count
            if vel_des is not None:
                for i, v in enumerate(vel_des[:3]):
                    msg.vel_des[i] = v
            if step_height is not None:
                for i, s in enumerate(step_height[:2]):
                    msg.step_height[i] = s
            self._cmd_msg = msg

    def stop(self):
        self.send_cmd(MODE_PURE_DAMPER, duration=4000)
        time.sleep(0.1)
        self._running = False

    def _heartbeat(self):
        while self._running:
            with self._lock:
                msg = self._cmd_msg
            if self._lcm_cmd and msg:
                self._lcm_cmd.publish(CMD_CHANNEL, msg.encode())
            time.sleep(0.1)


class Stage2OrangeScan(Node):
    STEP_HEIGHT = 0.06
    STAND_DUR = 1.5

    # Timed motions (first runnable version; can be replaced by odom/yaw closed loop later)
    TURN_RATE = 0.22
    TURN_180_DUR = 7.5
    GO_ORANGE_DUR = 1.8
    NEXT_ROW_SHIFT_DUR = 1.2

    MAX_ROWS = 4

    def __init__(self):
        super().__init__("stage2_orange_scan")
        self.state = State.INIT
        self._t0 = 0.0
        self._ctrl = RobotCtrl(self)
        self._row_idx = 0

        self._orange_seen = False
        self._orange_last_ts = 0.0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(String, "/ball_color", self._ball_cb, qos)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[S2] Stage2 orange scan ready")

    def _ball_cb(self, msg: String):
        color = msg.data.strip().lower()
        if color == "orange":
            self._orange_seen = True
            self._orange_last_ts = time.time()

    def control_loop(self):
        now = time.time()

        if self.state == State.INIT:
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            self.state = State.STAND_UP
            self._t0 = now
            self.get_logger().info("[S2] Recovery stand")
            return

        if self.state == State.STAND_UP:
            if now - self._t0 >= self.STAND_DUR:
                self.state = State.SCAN_TURN
                self._t0 = now
                self._orange_seen = False
                self.get_logger().info(f"[S2] Row {self._row_idx + 1}: scan by 180 turn")
            return

        if self.state == State.SCAN_TURN:
            # If orange detected during scan, go hit it immediately.
            if self._orange_seen:
                self.state = State.GO_TO_ORANGE
                self._t0 = now
                self.get_logger().info("[S2] Orange detected -> GO_TO_ORANGE")
                return

            if now - self._t0 >= self.TURN_180_DUR:
                self.state = State.SHIFT_NEXT_ROW
                self._t0 = now
                self.get_logger().info("[S2] Scan end -> SHIFT_NEXT_ROW")
                return

            # Slow left turn for scanning
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, self.TURN_RATE],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.GO_TO_ORANGE:
            if now - self._t0 >= self.GO_ORANGE_DUR:
                self.state = State.SHIFT_NEXT_ROW
                self._t0 = now
                self._orange_seen = False
                self.get_logger().info("[S2] Hit done -> SHIFT_NEXT_ROW")
                return

            # Move forward to hit the orange ball
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[0.20, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.SHIFT_NEXT_ROW:
            if now - self._t0 >= self.NEXT_ROW_SHIFT_DUR:
                self._row_idx += 1
                if self._row_idx >= self.MAX_ROWS:
                    self.state = State.DONE
                    self.get_logger().info("[S2] DONE")
                else:
                    self.state = State.SCAN_TURN
                    self._t0 = now
                    self._orange_seen = False
                    self.get_logger().info(f"[S2] Row {self._row_idx + 1}: scan by 180 turn")
                return

            # small side/forward adjustment to approximate "next row"
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.12, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.DONE:
            self._ctrl.send_cmd(MODE_LOCOMOTION, gait_id=GAIT_STAND, vel_des=[0.0, 0.0, 0.0])


def main(args=None):
    rclpy.init(args=args)
    node = Stage2OrangeScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._ctrl.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

