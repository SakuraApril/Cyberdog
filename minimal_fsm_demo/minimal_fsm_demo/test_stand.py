#!/usr/bin/env python3
"""Minimal test: damper first, then stand up. Same LCM path as stage5."""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node

try:
    import lcm
except Exception:
    lcm = None

robot_control_cmd_lcmt = None
localization_lcmt = None

local_dir = os.path.dirname(os.path.abspath(__file__))
if local_dir not in sys.path:
    sys.path.insert(0, local_dir)

try:
    from lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
    from lcm_types.localization_lcmt import localization_lcmt
except Exception:
    try:
        from minimal_fsm_demo.lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
        from minimal_fsm_demo.lcm_types.localization_lcmt import localization_lcmt
    except Exception:
        robot_control_cmd_lcmt = None
        localization_lcmt = None


LCM_CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
CMD_CHANNEL = "robot_control_cmd"

MODE_PURE_DAMPER = 7
MODE_RECOVERY_STAND = 12
CONTACT_ALL = 15


class RobotCtrl:
    def __init__(self, node):
        self._node = node
        self._lcm_cmd = None
        self._lock = threading.Lock()
        self._running = True
        self._life_count = 0
        self._cmd_msg = None
        self._last_key = None

        if lcm is None or robot_control_cmd_lcmt is None:
            self._node.get_logger().error("FATAL: LCM unavailable!")
            return

        self._cmd_msg = robot_control_cmd_lcmt()
        self._lcm_cmd = lcm.LCM(LCM_CMD_URL)
        self._node.get_logger().info(f"[Ctrl] LCM OK, url={LCM_CMD_URL}")
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def send_cmd(self, mode, gait_id=0, vel_des=None, rpy_des=None,
                 duration=0, contact=CONTACT_ALL, step_height=None):
        vkey = tuple(vel_des[:3]) if vel_des else (0.0, 0.0, 0.0)
        rkey = tuple(rpy_des[:3]) if rpy_des else (0.0, 0.0, 0.0)
        skey = tuple(step_height[:2]) if step_height else (0.07, 0.07)
        key = (mode, gait_id, vkey, rkey, duration, contact, skey)
        if key == self._last_key:
            return
        self._last_key = key

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
            if rpy_des is not None:
                for i, v in enumerate(rpy_des[:3]):
                    msg.rpy_des[i] = v
            if step_height is not None:
                for i, s in enumerate(step_height[:2]):
                    msg.step_height[i] = s
            self._cmd_msg = msg
            self._node.get_logger().info(
                f"[Ctrl] NEW CMD: mode={mode} life={self._life_count} dur={duration}")

    def stop(self):
        self.send_cmd(MODE_PURE_DAMPER, duration=2500)
        time.sleep(0.2)
        self._running = False

    def _heartbeat(self):
        count = 0
        while self._running:
            with self._lock:
                msg = self._cmd_msg
            if self._lcm_cmd and msg:
                self._lcm_cmd.publish(CMD_CHANNEL, msg.encode())
                count += 1
                if count == 1:
                    self._node.get_logger().info(
                        f"[Ctrl] PUB #1: mode={msg.mode} life={msg.life_count}")
                elif count % 50 == 0:
                    self._node.get_logger().info(
                        f"[Ctrl] PUB #{count}: mode={msg.mode} life={msg.life_count}")
            time.sleep(0.1)


class TestStand(Node):
    def __init__(self):
        super().__init__("test_stand")
        self._ctrl = RobotCtrl(self)
        self._state = "INIT"
        self._t0 = time.time()
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("==============================")
        self.get_logger().info("[Test] test_stand STARTING")
        self.get_logger().info("[Test] Step1: Lie down (damper)")
        self.get_logger().info("[Test] Step2: Stand up (recovery)")
        self.get_logger().info("==============================")

    def control_loop(self):
        now = time.time()

        if self._state == "INIT":
            # Step 1: lie down first so we can see the stand-up
            self._ctrl.send_cmd(MODE_PURE_DAMPER, duration=2000)
            self._state = "DAMP"
            self._t0 = now
            self.get_logger().info("[Test] >>> Sent DAMPER (lie down)")
            return

        if self._state == "DAMP":
            if now - self._t0 >= 2.5:
                # Step 2: stand up
                self._ctrl.send_cmd(
                    MODE_RECOVERY_STAND, contact=0, duration=5000,
                    step_height=[0.06, 0.06],
                )
                self._state = "STAND"
                self._t0 = now
                self.get_logger().info("[Test] >>> Sent RECOVERY STAND!")
            return

        if self._state == "STAND":
            if now - self._t0 >= 10.0:
                self._ctrl.stop()
                self.get_logger().info("[Test] Done")
                self._state = "DONE"
            return

        if self._state == "DONE":
            self._ctrl._running = False
            self.get_logger().info("[Test] Shutdown complete")
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TestStand()
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
