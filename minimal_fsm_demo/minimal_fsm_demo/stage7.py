#!/usr/bin/env python3
"""
Stage7: Jump forward 1m, then stop.
"""

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
LCM_RPY_URL = "udpm://239.255.76.67:7667?ttl=255"
CMD_CHANNEL = "robot_control_cmd"
RPY_CHANNEL = "global_to_robot"

MODE_PURE_DAMPER = 7
MODE_LOCOMOTION = 11
MODE_JUMP = 16
MODE_RECOVERY_STAND = 12
GAIT_STAND = 1
GAIT_TROT = 26
GAIT_JUMP = 1
CONTACT_ALL = 15


def normalize_angle(a):
    return a - (2 * math.pi) * math.floor((a + math.pi) / (2 * math.pi))


class RobotCtrl:
    def __init__(self, node):
        self._node = node
        self._lcm_cmd = None
        self._lcm_rpy = None
        self._lock = threading.Lock()
        self._running = True
        self._life_count = 0
        self._cmd_msg = None
        self._last_key = None

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.has_pose = False
        self.has_yaw = False

        if lcm is None or robot_control_cmd_lcmt is None:
            self._node.get_logger().error("LCM unavailable")
            return

        self._cmd_msg = robot_control_cmd_lcmt()
        self._lcm_cmd = lcm.LCM(LCM_CMD_URL)

        if localization_lcmt is not None:
            self._lcm_rpy = lcm.LCM(LCM_RPY_URL)
            self._lcm_rpy.subscribe(RPY_CHANNEL, self._on_rpy)
            threading.Thread(target=self._rpy_loop, daemon=True).start()

        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _on_rpy(self, channel, data):
        msg = localization_lcmt().decode(data)
        if hasattr(msg, "rpy") and len(msg.rpy) >= 3:
            self.yaw = float(msg.rpy[2])
            self.has_yaw = True
        if hasattr(msg, "xyz") and len(msg.xyz) >= 2:
            self.pos_x = float(msg.xyz[0])
            self.pos_y = float(msg.xyz[1])
            self.has_pose = True

    def _rpy_loop(self):
        while self._running:
            try:
                self._lcm_rpy.handle()
            except Exception:
                pass
            time.sleep(0.02)

    def send_cmd(self, mode, gait_id=0, vel_des=None, rpy_des=None,
                 duration=0, contact=CONTACT_ALL, step_height=None):
        vkey = tuple(vel_des[:3]) if vel_des else (0.0, 0.0, 0.0)
        rkey = tuple(rpy_des[:3]) if rpy_des else (0.0, 0.0, 0.0)
        skey = tuple(step_height[:2]) if step_height else (0.06, 0.06)
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

    def stop(self):
        self.send_cmd(MODE_PURE_DAMPER, duration=2500)
        time.sleep(0.1)
        self._running = False

    def _heartbeat(self):
        while self._running:
            with self._lock:
                msg = self._cmd_msg
            if self._lcm_cmd and msg:
                self._lcm_cmd.publish(CMD_CHANNEL, msg.encode())
            time.sleep(0.1)


class Stage7Jump(Node):
    STAND_DUR = 2.0
    TARGET_DIST = 1.00
    JUMP_SPEED_FWD = 0.70
    JUMP_TIMEOUT = 6.0
    FALLBACK_JUMP_DUR = 1.2

    def __init__(self):
        super().__init__("stage7_jump")
        self._ctrl = RobotCtrl(self)
        self._state = "INIT"
        self._t0 = time.time()
        self._jump_start_t = 0.0
        self._start_x = 0.0
        self._start_y = 0.0
        self._start_yaw = 0.0
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("[S7] Jump forward target: 1.00m")

    def _fwd_progress(self):
        if not self._ctrl.has_pose:
            return None
        dx = self._ctrl.pos_x - self._start_x
        dy = self._ctrl.pos_y - self._start_y
        if self._ctrl.has_yaw:
            # body-forward projection:
            # forward = [cos(yaw), sin(yaw)] in world frame
            return dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        return math.hypot(dx, dy)

    def control_loop(self):
        now = time.time()

        if self._state == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[0.06, 0.06],
            )
            self._state = "STAND"
            self._t0 = now
            self.get_logger().info("[S7] Recovery stand")
            return

        if self._state == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                self._start_x = self._ctrl.pos_x
                self._start_y = self._ctrl.pos_y
                self._start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else 0.0
                self._jump_start_t = now
                self._state = "JUMP"
                self.get_logger().info(
                    f"[S7] Start jump from ({self._start_x:.3f}, {self._start_y:.3f})")
            return

        if self._state == "JUMP":
            progress = self._fwd_progress()
            elapsed = now - self._jump_start_t
            done_by_dist = progress is not None and progress >= self.TARGET_DIST
            done_by_timeout = elapsed >= self.JUMP_TIMEOUT
            done_by_fallback = (not self._ctrl.has_pose) and elapsed >= self.FALLBACK_JUMP_DUR

            if done_by_dist or done_by_timeout or done_by_fallback:
                self._state = "STOP"
                reason = "distance" if done_by_dist else (
                    "fallback-time" if done_by_fallback else "timeout")
                if progress is not None:
                    self.get_logger().info(
                        f"[S7] Jump done by {reason}, progress={progress:.3f}m")
                else:
                    self.get_logger().info(f"[S7] Jump done by {reason}")
                return

            self._ctrl.send_cmd(
                MODE_JUMP, gait_id=GAIT_JUMP,
                vel_des=[self.JUMP_SPEED_FWD, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[0.06, 0.06],
            )
            return

        if self._state == "STOP":
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_STAND,
                vel_des=[0.0, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[0.06, 0.06],
            )
            self._state = "DONE"
            self._t0 = now
            return

        if self._state == "DONE":
            if now - self._t0 >= 1.2:
                final_progress = self._fwd_progress()
                if final_progress is not None:
                    self.get_logger().info(f"[S7] Final forward distance: {final_progress:.3f}m")
                self.get_logger().info("[S7] Completed.")
                self._ctrl.stop()
                raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Stage7Jump()
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
