#!/usr/bin/env python3
"""
Navigate to target then additional legs. Open-loop, no TF.

Leg 1: left 2.8m, forward 1.04m -> turn left 69.6°, walk 2.987m
Leg 2: turn left 127°, walk 119cm
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node

try:
    import lcm
except Exception:
    lcm = None

try:
    from minimal_fsm_demo.lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
except Exception:
    robot_control_cmd_lcmt = None

LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
LCM_CHANNEL = "robot_control_cmd"

MODE_PURE_DAMPER = 7
MODE_LOCOMOTION = 11
MODE_RECOVERY_STAND = 12
GAIT_STAND = 1
GAIT_TROT = 26
CONTACT_ALL = 15

# ── Target (robot frame) ──
TARGET_X = 1.04    # forward (m)
TARGET_Y = 2.8     # left (m)

# ── Calibrated values ──
WALK_SPEED = 0.3          # m/s
YAW_RATE = 1.3            # rad/s (positive = left)
TURN_90_DUR = 1.55        # 90° = 1.55s (from motion_ctrl.py)
STEP_HEIGHT = 0.06
STAND_DUR = 5.0

# Pre-calculate
TARGET_DIST = math.sqrt(TARGET_X**2 + TARGET_Y**2)
TARGET_ANGLE_DEG = math.degrees(math.atan2(TARGET_Y, TARGET_X))
TURN_DUR = (TARGET_ANGLE_DEG / 90.0) * TURN_90_DUR
WALK_DUR = TARGET_DIST / WALK_SPEED

# ── Leg 2 ──
LEG2_ANGLE_DEG = -127.0       # turn left 127°
LEG2_DIST = 1.5             # walk 119cm
LEG2_TURN_DUR = abs(LEG2_ANGLE_DEG / 90.0) * TURN_90_DUR
LEG2_WALK_DUR = LEG2_DIST / WALK_SPEED


class RobotCtrl:
    def __init__(self, node):
        self._node = node
        self._lcm = None
        self._lock = threading.Lock()
        self._cmd_msg = robot_control_cmd_lcmt()
        self._running = True
        self._life_count = 0
        if lcm is None or robot_control_cmd_lcmt is None:
            self._node.get_logger().error("LCM unavailable!")
            return
        self._lcm = lcm.LCM(LCM_URL)
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def send_cmd(self, mode, gait_id=0, vel_des=None, duration=0,
                 contact=CONTACT_ALL, step_height=None):
        with self._lock:
            self._life_count = (self._life_count + 1) % 128
            msg = robot_control_cmd_lcmt()
            msg.mode = mode
            msg.gait_id = gait_id
            msg.contact = contact
            msg.duration = duration
            msg.life_count = self._life_count
            if vel_des:
                for i, v in enumerate(vel_des[:3]):
                    msg.vel_des[i] = v
            if step_height:
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
            if self._lcm and msg:
                self._lcm.publish(LCM_CHANNEL, msg.encode())
            time.sleep(0.1)


class GotoTarget(Node):

    def __init__(self):
        super().__init__("goto_target")
        self._ctrl = RobotCtrl(self)
        self._phase = "INIT"
        self._t0 = 0.0
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"[Goto] Target: left {TARGET_Y}m, forward {TARGET_X}m")
        self.get_logger().info(
            f"[Goto] Angle={TARGET_ANGLE_DEG:.1f}°  "
            f"Dist={TARGET_DIST:.3f}m  "
            f"Turn={TURN_DUR:.2f}s  Walk={WALK_DUR:.2f}s")

    def control_loop(self):
        now = time.time()

        if self._phase == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[STEP_HEIGHT, STEP_HEIGHT])
            self._phase = "STAND"
            self._t0 = now
            self.get_logger().info("[Goto] Recovery stand")
            return

        if self._phase == "STAND":
            if now - self._t0 >= STAND_DUR:
                self._phase = "TURN"
                self._t0 = now
                self.get_logger().info(
                    f"[Goto] Turn left {TARGET_ANGLE_DEG:.1f}° ({TURN_DUR:.2f}s)")
            return

        if self._phase == "TURN":
            if now - self._t0 >= TURN_DUR:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_STAND,
                    vel_des=[0.0, 0.0, 0.0])
                self._phase = "WALK"
                self._t0 = now
                self.get_logger().info(
                    f"[Goto] Walk forward {TARGET_DIST:.3f}m ({WALK_DUR:.2f}s)")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[0.0, 0.0, YAW_RATE],
                step_height=[STEP_HEIGHT, STEP_HEIGHT])
            return

        if self._phase == "WALK":
            if now - self._t0 >= WALK_DUR:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_STAND,
                    vel_des=[0.0, 0.0, 0.0])
                self._phase = "TURN2"
                self._t0 = now
                self.get_logger().info(
                    f"[Goto] Turn right {abs(LEG2_ANGLE_DEG):.0f}° ({LEG2_TURN_DUR:.2f}s)")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[WALK_SPEED, 0.0, 0.0],
                step_height=[STEP_HEIGHT, STEP_HEIGHT])
            return

        if self._phase == "TURN2":
            if now - self._t0 >= LEG2_TURN_DUR:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_STAND,
                    vel_des=[0.0, 0.0, 0.0])
                self._phase = "WALK2"
                self._t0 = now
                self.get_logger().info(
                    f"[Goto] Walk {LEG2_DIST:.2f}m ({LEG2_WALK_DUR:.2f}s)")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[0.0, 0.0, -YAW_RATE],
                step_height=[STEP_HEIGHT, STEP_HEIGHT])
            return

        if self._phase == "WALK2":
            if now - self._t0 >= LEG2_WALK_DUR:
                self._ctrl.stop()
                self.get_logger().info("[Goto] DONE!")
                self._phase = "DONE"
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[WALK_SPEED, 0.0, 0.0],
                step_height=[STEP_HEIGHT, STEP_HEIGHT])
            return

        if self._phase == "DONE":
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GotoTarget()
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
