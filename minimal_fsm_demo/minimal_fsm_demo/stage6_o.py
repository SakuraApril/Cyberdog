#!/usr/bin/env python3
"""
Stage6: Straight → right turn → straight → right turn 135° → sit.
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
GAIT_TROT_SLOW = 27
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
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.has_pose = False
        self.has_yaw = False
        self._last_key = None
        self._cal_ok = False
        self._cal_x = 0.0
        self._cal_y = 0.0
        self._cal_yaw = 0.0
        self._cal_yaw_ok = False

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
            raw_yaw = float(msg.rpy[2])
            if self._cal_yaw_ok:
                self.yaw = normalize_angle(raw_yaw + self._cal_yaw)
            else:
                self.yaw = raw_yaw
            self.has_yaw = True
        if hasattr(msg, "xyz") and len(msg.xyz) >= 2:
            raw_x = float(msg.xyz[0])
            raw_y = float(msg.xyz[1])
            if self._cal_ok:
                self.pos_x = raw_x + self._cal_x
                self.pos_y = raw_y + self._cal_y
            else:
                self.pos_x = raw_x
                self.pos_y = raw_y
            self.has_pose = True

    def set_calibration(self, ref_x, ref_y, ref_yaw=None):
        with self._lock:
            self._cal_x = ref_x - self.pos_x
            self._cal_y = ref_y - self.pos_y
            self._cal_ok = True
            self.pos_x = ref_x
            self.pos_y = ref_y
            self._node.get_logger().info(
                f"[Ctrl] Cal pos: offset=({self._cal_x:.3f}, {self._cal_y:.3f})")
            if ref_yaw is not None and self.has_yaw:
                self._cal_yaw = normalize_angle(ref_yaw - self.yaw)
                self._cal_yaw_ok = True
                self.yaw = ref_yaw
                self._node.get_logger().info(
                    f"[Ctrl] Cal yaw: offset={self._cal_yaw:.3f}rad "
                    f"({math.degrees(self._cal_yaw):.1f}deg)")

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


class Stage6Nav(Node):
    START_X = 2.5
    START_Y = 13.50052
    START_YAW = 3.08662
    WP1_X = 0.2
    WP1_Y = 13.50052  # straight forward from start
    WP2_X = 0.2
    WP2_Y = 14.90052  # after right turn, walk to here
    WP3_X = 1.55
    WP3_Y = 13.55052
    TARGET_X = 2.3
    TARGET_X2 = 2.7
    WALK_SPEED = 0.25
    WALK_STEP = 0.08
    TURN_SPEED = 0.35
    YAW_CORRECT_SPEED = 0.15
    TURN_TOL_DEG = 4.0
    STAND_DUR = 8.0
    JUMP_TARGET_DIST = 1.0
    JUMP_SPEED_FWD = 0.70
    JUMP_TIMEOUT = 6.0
    JUMP_FALLBACK_DUR = 1.2
    JUMP_SETTLE_DUR = 0.8
    POST_JUMP_RECOVERY_DUR = 1.6
    TURN1_YAW = math.pi / 2         # 正朝右 (+Y)
    TURN2_YAW = -0.7             # ~ -40°
    ANTI_SLIDE_VY = 0.03

    def __init__(self):
        super().__init__("stage6_nav")
        self._ctrl = RobotCtrl(self)
        self._state = "INIT"
        self._t0 = time.time()
        self._turn_target = 0.0
        self._turn_start_yaw = None
        self._calibrated = False
        self._jump_start_t = 0.0
        self._jump_start_x = 0.0
        self._jump_start_y = 0.0
        self._jump_start_yaw = 0.0
        self._last_walk_debug_t = 0.0
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("==============================")
        self.get_logger().info("[S6] Stage6 Nav")
        self.get_logger().info(f"[S6] Start: ({self.START_X:.3f}, {self.START_Y:.3f}) yaw={self.START_YAW:.3f}")
        self.get_logger().info(f"[S6] WP1: ({self.WP1_X:.3f}, {self.WP1_Y:.3f})")
        self.get_logger().info(f"[S6] WP2: ({self.WP2_X:.3f}, {self.WP2_Y:.3f})")
        self.get_logger().info(f"[S6] Walk to x={self.TARGET_X} → yaw=0 → x={self.TARGET_X2} → sit")
        self.get_logger().info("==============================")

    def _turned_deg(self):
        if self._turn_start_yaw is None or not self._ctrl.has_yaw:
            return None
        return math.degrees(normalize_angle(self._ctrl.yaw - self._turn_start_yaw))

    def _distance_to(self, tx, ty):
        if not self._ctrl.has_pose:
            return None
        return math.sqrt((self._ctrl.pos_x - tx) ** 2 + (self._ctrl.pos_y - ty) ** 2)

    def _heading_to(self, tx, ty):
        if not self._ctrl.has_pose:
            return None
        return math.atan2(ty - self._ctrl.pos_y, tx - self._ctrl.pos_x)

    def _jump_forward_progress(self):
        if not self._ctrl.has_pose:
            return None
        dx = self._ctrl.pos_x - self._jump_start_x
        dy = self._ctrl.pos_y - self._jump_start_y
        # 使用起跳时朝向做前向投影，避免横向漂移影响里程判断
        return dx * math.cos(self._jump_start_yaw) + dy * math.sin(self._jump_start_yaw)

    def control_loop(self):
        now = time.time()

        # Keep calibrating yaw until success
        if not self._ctrl._cal_yaw_ok and self._ctrl.has_pose and self._ctrl.has_yaw:
            self._ctrl.set_calibration(self.START_X, self.START_Y, ref_yaw=self.START_YAW)
        if not self._calibrated and self._ctrl._cal_ok:
            self._calibrated = True

        # ── INIT: Recovery stand ──
        if self._state == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[0.06, 0.06],
            )
            self._state = "STAND"
            self._t0 = now
            self.get_logger().info("[S6] Recovery stand...")
            return

        # ── STAND: wait ──
        if self._state == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                self.get_logger().info(
                    f"[S6] Stand done. yaw_ok={self._ctrl._cal_yaw_ok} → JUMP_START")
                self._state = "JUMP_START"
                self._t0 = now
            return

        # ── JUMP_START: record initial pose for 1m forward jump ──
        if self._state == "JUMP_START":
            self._jump_start_x = self._ctrl.pos_x
            self._jump_start_y = self._ctrl.pos_y
            self._jump_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else self.START_YAW
            self._jump_start_t = now
            self._state = "JUMP_FWD"
            self.get_logger().info(
                f"[S6] Start jump forward 1.0m from ({self._jump_start_x:.3f}, {self._jump_start_y:.3f})")
            return

        # ── JUMP_FWD: jump forward until 1m or timeout ──
        if self._state == "JUMP_FWD":
            progress = self._jump_forward_progress()
            elapsed = now - self._jump_start_t
            done_by_dist = progress is not None and progress >= self.JUMP_TARGET_DIST
            done_by_timeout = elapsed >= self.JUMP_TIMEOUT
            done_by_fallback = (not self._ctrl.has_pose) and elapsed >= self.JUMP_FALLBACK_DUR

            if done_by_dist or done_by_timeout or done_by_fallback:
                reason = "distance" if done_by_dist else ("fallback-time" if done_by_fallback else "timeout")
                if progress is not None:
                    self.get_logger().info(
                        f"[S6] Jump done by {reason}, progress={progress:.3f}m")
                else:
                    self.get_logger().info(f"[S6] Jump done by {reason}")
                self._state = "JUMP_SETTLE"
                self._t0 = now
                return

            self._ctrl.send_cmd(
                MODE_JUMP, gait_id=GAIT_JUMP,
                vel_des=[self.JUMP_SPEED_FWD, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[0.06, 0.06],
            )
            return

        # ── JUMP_SETTLE: stand briefly after jump then continue stage6 path ──
        if self._state == "JUMP_SETTLE":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[0.06, 0.06],
            )
            if now - self._t0 >= self.POST_JUMP_RECOVERY_DUR:
                self._state = "WALK_TO_WP1"
                self._t0 = now
                self.get_logger().info("[S6] Jump recovery done → WALK_TO_WP1")
            return

        # ── WALK_TO_WP1: straight to (0.2, 13.50052) ──
        if self._state == "WALK_TO_WP1":
            dist = self._distance_to(self.WP1_X, self.WP1_Y)
            if dist is not None and dist <= 0.25:
                self.get_logger().info(
                    f"[S6] Reached WP1 (dist={dist:.3f}), turn to yaw={self.TURN1_YAW:.3f}")
                if self._ctrl.has_yaw:
                    self._turn_target = math.degrees(
                        normalize_angle(self.TURN1_YAW - self._ctrl.yaw))
                    self._turn_start_yaw = self._ctrl.yaw
                else:
                    self._turn_target = -90.0
                    self._turn_start_yaw = None
                self._state = "TURN1"
                self._t0 = now
                return
            # Walk toward WP1 with heading correction
            if self._ctrl.has_pose and self._ctrl._cal_yaw_ok:
                target = self._heading_to(self.WP1_X, self.WP1_Y)
                err = normalize_angle(target - self._ctrl.yaw)
                wz = max(-0.3, min(0.3, err * 1.5))
            else:
                wz = 0.0
            if now - self._last_walk_debug_t >= 1.0:
                self._last_walk_debug_t = now
                self.get_logger().info(
                    f"[S6] WALK_TO_WP1 running: dist={dist}, "
                    f"pose=({self._ctrl.pos_x:.3f},{self._ctrl.pos_y:.3f}), "
                    f"yaw={self._ctrl.yaw:.3f}, wz={wz:.3f}")
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── TURN1: turn to face 正朝右 (+Y) ──
        if self._state == "TURN1":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S6] Turn1 done: {turned:.1f}deg → WALK_TO_WP2")
                self._state = "WALK_TO_WP2"
                self._t0 = now
                return
            wz = self.TURN_SPEED if target > 0 else -self.TURN_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── WALK_TO_WP2: straight to (0.2, 14.90052) ──
        if self._state == "WALK_TO_WP2":
            dist = self._distance_to(self.WP2_X, self.WP2_Y)
            if dist is not None and dist <= 0.2:
                self.get_logger().info(
                    f"[S6] Reached WP2 (dist={dist:.3f}), turn to yaw={self.TURN2_YAW:.3f}")
                if self._ctrl.has_yaw:
                    self._turn_target = math.degrees(
                        normalize_angle(self.TURN2_YAW - self._ctrl.yaw))
                    self._turn_start_yaw = self._ctrl.yaw
                else:
                    self._turn_target = -135.0
                    self._turn_start_yaw = None
                self._state = "TURN2"
                self._t0 = now
                return
            if self._ctrl.has_pose and self._ctrl._cal_yaw_ok:
                target = self._heading_to(self.WP2_X, self.WP2_Y)
                err = normalize_angle(target - self._ctrl.yaw)
                wz = max(-0.3, min(0.3, err * 1.5))
            else:
                wz = 0.0
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── TURN2: turn to -π/4 (右下45°) ──
        if self._state == "TURN2":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S6] Turn2 done: {turned:.1f}deg → WALK_WP3 ({self.WP3_X:.2f},{self.WP3_Y:.2f})")
                self._state = "WALK_WP3"
                self._t0 = now
                return
            wz = self.TURN_SPEED if target > 0 else -self.TURN_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── WALK_WP3: walk until x >= TARGET_X ──
        if self._state == "WALK_WP3":
            if self._ctrl.has_pose and self._ctrl.pos_x >= self.TARGET_X:
                self.get_logger().info(
                    f"[S6] Reached x={self._ctrl.pos_x:.3f} → turn to yaw=0")
                if self._ctrl.has_yaw:
                    turn_deg = math.degrees(
                        normalize_angle(0.0 - self._ctrl.yaw))
                    self._turn_target = turn_deg
                    self._turn_start_yaw = self._ctrl.yaw
                else:
                    self._turn_target = 0.0
                    self._turn_start_yaw = None
                self._state = "TURN_WP4"
                self._t0 = now
                return
            if self._ctrl.has_pose and self._ctrl._cal_yaw_ok:
                target = self._heading_to(self.WP3_X, self.WP3_Y)
                err = normalize_angle(target - self._ctrl.yaw)
                wz = max(-0.3, min(0.3, err * 1.5))
            else:
                wz = 0.0
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── TURN_WP4: turn to yaw=0 (正右 East) ──
        if self._state == "TURN_WP4":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S6] Turn WP4 done: {turned:.1f}deg → walk to x=2.4")
                self._state = "WALK_X24"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── WALK_X24: walk until x >= TARGET_X2 → sit ──
        if self._state == "WALK_X24":
            if self._ctrl.has_pose and self._ctrl.pos_x >= self.TARGET_X2:
                self.get_logger().info(
                    f"[S6] Reached x={self._ctrl.pos_x:.3f} → SIT DOWN")
                self._state = "SIT_DOWN"
                self._t0 = now
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # ── SIT_DOWN: damper ──
        if self._state == "SIT_DOWN":
            self._ctrl.send_cmd(MODE_PURE_DAMPER, duration=3000)
            self.get_logger().info("[S6] Damper (sit down)...")
            self._state = "WAIT_SIT"
            self._t0 = now
            return

        if self._state == "WAIT_SIT":
            if now - self._t0 >= 3.0:
                self._ctrl.stop()
                self.get_logger().info("[S6] Stage6 complete!")
                self._state = "DONE"
            return

        if self._state == "DONE":
            self._ctrl._running = False
            self.get_logger().info("[S6] Shutdown")
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Stage6Nav()
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
