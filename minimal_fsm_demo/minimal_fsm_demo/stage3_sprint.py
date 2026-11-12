#!/usr/bin/env python3
"""
Stage 3: 曲道冲锋 - waypoint navigation.
Walk to 3 target coordinates sequentially via straight-line segments,
with final yaw alignment at the last waypoint.
"""

import math
import os
import subprocess
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
MODE_RECOVERY_STAND = 12
GAIT_STAND = 1
GAIT_TROT = 26
GAIT_TROT_SLOW = 27
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
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.has_pose = False
        self.has_yaw = False
        self._last_key = None

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
        threading.Thread(target=self._pos_loop, daemon=True).start()

    def _pos_loop(self):
        """Read Gazebo ground-truth position (world frame, matches teleport coords)."""
        while self._running:
            try:
                result = subprocess.run(
                    ['gz', 'model', '-m', 'robot', '-p'],
                    capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    parts = result.stdout.strip().split()
                    if len(parts) >= 2:
                        self.pos_x = float(parts[0])
                        self.pos_y = float(parts[1])
                        self.has_pose = True
            except Exception:
                pass
            time.sleep(0.1)

    def _on_rpy(self, channel, data):
        msg = localization_lcmt().decode(data)
        if hasattr(msg, "rpy") and len(msg.rpy) >= 3:
            self.yaw = float(msg.rpy[2])
            self.has_yaw = True

    def _rpy_loop(self):
        while self._running:
            try:
                self._lcm_rpy.handle()
            except Exception:
                pass
            time.sleep(0.02)

    def send_cmd(self, mode, gait_id=0, vel_des=None, duration=0, contact=CONTACT_ALL, step_height=None):
        # Skip if identical to last command (avoids resetting gait controller)
        vkey = tuple(vel_des[:3]) if vel_des else (0.0, 0.0, 0.0)
        skey = tuple(step_height[:2]) if step_height else (0.06, 0.06)
        key = (mode, gait_id, vkey, duration, contact, skey)
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


class Stage3Waypoints(Node):
    STAND_DUR = 1.5
    STEP_HEIGHT = 0.06
    WALK_SPEED = 0.20
    TURN_SPEED_LEFT = 0.3
    TURN_SPEED_RIGHT = -0.3
    TURN_TOL_DEG = 3.0
    WALK_TOL_M = 0.05

    WAYPOINTS = [
        (0.350854, 5.40052),
        (2.350854, 5.80052),
        (3.050854, 6.50052),
    ]
    FINAL_YAW = 1.51582
    START_X = -0.350854
    START_Y = 4.30052
    START_YAW = 1.51582

    def __init__(self):
        super().__init__("stage3_waypoints")
        self._ctrl = RobotCtrl(self)
        self._state = "INIT"
        self._t0 = time.time()
        self._wp_index = -1
        self._phase = None          # "turn" | "walk" | "final_yaw"
        self._seg_target = 0.0
        self._seg_start_pos = None
        self._seg_start_yaw = None
        self._seg_dist = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"[S3] Start=({self.START_X:.3f},{self.START_Y:.3f}) yaw={self.START_YAW:.3f}"
        )
        self.get_logger().info(
            f"[S3] WP1={self.WAYPOINTS[0]} WP2={self.WAYPOINTS[1]} WP3={self.WAYPOINTS[2]}"
        )

    def _compute_segment(self, target_x, target_y):
        if not self._ctrl.has_pose or not self._ctrl.has_yaw:
            return None, None
        dx = target_x - self._ctrl.pos_x
        dy = target_y - self._ctrl.pos_y
        dist = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        turn = math.degrees(normalize_angle(target_heading - self._ctrl.yaw))
        return turn, dist

    def _forward_progress(self):
        if self._seg_start_pos is None or not self._ctrl.has_pose:
            return None
        dx = self._ctrl.pos_x - self._seg_start_pos[0]
        dy = self._ctrl.pos_y - self._seg_start_pos[1]
        if self._seg_start_yaw is not None:
            return dx * math.cos(self._seg_start_yaw) + dy * math.sin(self._seg_start_yaw)
        return math.hypot(dx, dy)

    def _turned_deg(self):
        if self._seg_start_yaw is None or not self._ctrl.has_yaw:
            return None
        return math.degrees(normalize_angle(self._ctrl.yaw - self._seg_start_yaw))

    def _advance_wp(self):
        self._wp_index += 1
        if self._wp_index >= len(self.WAYPOINTS):
            turn = math.degrees(normalize_angle(self.FINAL_YAW - self._ctrl.yaw))
            self._phase = "final_yaw"
            self._seg_target = turn
            self._seg_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
            self.get_logger().info(f"[S3] Final yaw align: {turn:.1f}deg")
            return

        tx, ty = self.WAYPOINTS[self._wp_index]
        turn_deg, dist = self._compute_segment(tx, ty)
        if turn_deg is None or dist is None:
            self.get_logger().warn("[S3] No pose, retrying in 0.5s")
            self._wp_index -= 1
            self._state = "WAIT_POSE"
            self._t0 = time.time()
            return

        self._phase = "turn"
        self._seg_target = turn_deg
        self._seg_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
        self._seg_dist = dist
        self.get_logger().info(
            f"[S3] WP{self._wp_index + 1}: turn {turn_deg:.1f}deg, walk {dist:.3f}m → ({tx:.3f}, {ty:.3f})")

    def control_loop(self):
        now = time.time()

        if self._state == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT],
            )
            self._state = "STAND"
            self._t0 = now
            self.get_logger().info("[S3] Recovery stand")
            return

        if self._state == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                self._advance_wp()
                self._state = "EXEC"
            return

        if self._state == "WAIT_POSE":
            if now - self._t0 >= 0.5:
                self._advance_wp()
                self._state = "EXEC"
            return

        if self._state == "EXEC":
            if self._phase == "turn":
                turned = self._turned_deg()
                target = self._seg_target
                done = False
                if turned is not None:
                    if target > 0 and turned >= target - self.TURN_TOL_DEG:
                        done = True
                    elif target < 0 and turned <= target + self.TURN_TOL_DEG:
                        done = True
                if done:
                    self.get_logger().info(f"[S3] Turn done: {turned:.1f}deg")
                    self._phase = "walk"
                    self._seg_target = self._seg_dist
                    self._seg_start_pos = (self._ctrl.pos_x, self._ctrl.pos_y) if self._ctrl.has_pose else None
                    self._seg_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                    return

                wz = self.TURN_SPEED_LEFT if target > 0 else self.TURN_SPEED_RIGHT
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                    vel_des=[0.0, 0.0, wz],
                    step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT],
                )
                return

            if self._phase == "walk":
                proj = self._forward_progress()
                if proj is not None and proj >= self._seg_target - self.WALK_TOL_M:
                    self.get_logger().info(f"[S3] Walk done: {proj:.3f}m")
                    self._advance_wp()
                    return
                if proj is not None:
                    remaining = max(0.0, self._seg_target - proj)
                    speed = max(0.06, self.WALK_SPEED * min(1.0, remaining / 0.30))
                else:
                    speed = self.WALK_SPEED
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_TROT,
                    vel_des=[speed, 0.0, 0.0],
                    step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT],
                )
                return

            if self._phase == "final_yaw":
                turned = self._turned_deg()
                target = self._seg_target
                done = False
                if turned is not None:
                    if target > 0 and turned >= target - self.TURN_TOL_DEG:
                        done = True
                    elif target < 0 and turned <= target + self.TURN_TOL_DEG:
                        done = True
                if done:
                    self.get_logger().info(f"[S3] Final yaw done: {turned:.1f}deg")
                    self._state = "DONE"
                    return

                wz = self.TURN_SPEED_LEFT if target > 0 else self.TURN_SPEED_RIGHT
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                    vel_des=[0.0, 0.0, wz],
                    step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT],
                )
                return

        if self._state == "DONE":
            self._ctrl.send_cmd(MODE_LOCOMOTION, gait_id=GAIT_STAND, vel_des=[0.0, 0.0, 0.0])
            time.sleep(0.5)
            self._ctrl.stop()
            self._ctrl._running = False
            self.get_logger().info("[S3] Shutdown complete")
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Stage3Waypoints()
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
