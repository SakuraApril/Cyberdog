#!/usr/bin/env python3
"""
Stage5: 直走5米 with squatting posture.
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
MODE_RECOVERY_STAND = 12
MODE_JUMP = 16
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
            raw_x = float(msg.xyz[0])
            raw_y = float(msg.xyz[1])
            if self._cal_ok:
                self.pos_x = raw_x + self._cal_x
                self.pos_y = raw_y + self._cal_y
            else:
                self.pos_x = raw_x
                self.pos_y = raw_y
            self.has_pose = True

    def set_calibration(self, ref_x, ref_y):
        with self._lock:
            self._cal_x = ref_x - self.pos_x
            self._cal_y = ref_y - self.pos_y
            self._cal_ok = True
            self.pos_x = ref_x
            self.pos_y = ref_y
            self._node.get_logger().info(
                f"[Ctrl] Calibrated: offset=({self._cal_x:.3f}, {self._cal_y:.3f})")

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


class Stage5Walk(Node):
    STAND_DUR = 10.0
    # 上坡参数（身体水平 + 腿抬高）
    CLIMB_SPEED = 0.3
    CLIMB_DUR = 13.0
    CLIMB_STEP = 0.08
    # 直走参数（身体水平 + 腿抬高）
    WALK_SPEED = 0.30
    WALK_STEP = 0.08
    # 转弯参数
    TURN_SPEED = 0.7
    YAW_CORRECT_SPEED = 0.15  # 角度校正用慢速
    TURN_TOL_DEG = 3.0
    # 坐标
    START_X = 3.1
    START_Y = 7.20052
    START_YAW = 1.51582
    CORRECT_Y = 7.5          # 在y=7.5处校正角度
    TARGET_Y = 12.3
    ANTI_SLIDE_VY = 0.07      # 斜坡防右滑，vy>0 往左推
    X_TARGET = 3.0            # 后退爬坡时保持x坐标
    X_KP = 0.3                # x校正P增益
    POS_STABLE_NEEDED = 3     # 连续3次(0.15s)满足条件才触发

    def __init__(self):
        super().__init__("stage5_walk")
        self._ctrl = RobotCtrl(self)
        self._state = "INIT"
        self._t0 = time.time()
        self._turn_target = 0.0
        self._turn_start_yaw = None
        self._calibrated = False
        self._pos_stable = 0     # debounce counter for y-position triggers
        self._jump_start_t = 0.0
        self._jump_start_x = 0.0
        self._jump_start_y = 0.0
        self._jump_start_yaw = 0.0
        self._walk_8cm_start_x = 0.0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"[S5] Start=({self.START_X:.3f},{self.START_Y:.3f}), "
            f"climb {self.CLIMB_DUR}s → turn right → walk to y={self.TARGET_Y}")

    def _turned_deg(self):
        if self._turn_start_yaw is None or not self._ctrl.has_yaw:
            return None
        return math.degrees(normalize_angle(self._ctrl.yaw - self._turn_start_yaw))

    def _x_correct_vy(self, target=None):
        """Lateral vy to push x toward target (default X_TARGET)."""
        if not self._ctrl.has_pose:
            return 0.0
        tgt = target if target is not None else self.X_TARGET
        x_err = tgt - self._ctrl.pos_x
        vy = x_err * self.X_KP
        return max(-0.12, min(0.12, vy))

    def _anti_slide_vy(self):
        """Anti-slide lateral push, disabled when y <= 12.3."""
        if not self._ctrl.has_pose or self._ctrl.pos_y <= 12.3:
            return 0.0
        return self.ANTI_SLIDE_VY

    def _y_correct_vy(self, target):
        """Lateral vy to push y toward target."""
        if not self._ctrl.has_pose:
            return 0.0
        y_err = target - self._ctrl.pos_y
        vy = y_err * self.X_KP
        return max(-0.05, min(0.05, vy))

    def _get_roll(self):
        """Roll=0.05 only when y >= 12.4, else 0.0."""
        if self._ctrl.has_pose and self._ctrl.pos_y >= 12.4:
            return 0.05
        return 0.0

    def control_loop(self):
        now = time.time()

        if not self._calibrated and self._ctrl.has_pose:
            self._ctrl.set_calibration(self.START_X, self.START_Y)
            self._calibrated = True

        if self._state == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[0.06, 0.06],
            )
            self._state = "STAND"
            self._t0 = now
            self.get_logger().info("[S5] Recovery stand")
            return

        if self._state == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                if self._ctrl.has_yaw:
                    self._turn_target = 180.0
                    self._turn_start_yaw = self._ctrl.yaw
                    self._state = "TURN_180"
                    self.get_logger().info("[S5] Stand done, turning 180°...")
                else:
                    self._state = "BACKWALK"
                    self._t0 = now
                    self.get_logger().info("[S5] Stand done, no yaw, backwalking...")
            return

        if self._state == "TURN_180":
            turned = self._turned_deg()
            target = self._turn_target
            done = turned is not None and turned >= target - self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Turn 180 done: {turned:.1f}deg, backwalking...")
                self._state = "BACKWALK"
                self._t0 = now
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, self.TURN_SPEED],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        if self._state == "BACKWALK":
            if self._ctrl.has_pose and self._ctrl.pos_y >= self.CORRECT_Y:
                self._pos_stable += 1
            else:
                self._pos_stable = 0
            if self._pos_stable >= self.POS_STABLE_NEEDED:
                if self._ctrl.has_yaw:
                    turn_deg = math.degrees(normalize_angle(-1.57 - self._ctrl.yaw))
                    self._turn_target = turn_deg
                    self._turn_start_yaw = self._ctrl.yaw
                    self._state = "YAW_CORRECT"
                    self._pos_stable = 0
                    self.get_logger().info(f"[S5] Reached y={self._ctrl.pos_y:.3f}, yaw correct {turn_deg:.1f}deg...")
                else:
                    self._state = "BACKWALK2"
                    self._pos_stable = 0
                    self._t0 = now
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[-self.WALK_SPEED, self._x_correct_vy(), 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "YAW_CORRECT":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw correct done: {turned:.1f}deg")
                self._state = "BACKWALK2"
                self._t0 = now
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        if self._state == "BACKWALK2":
            if self._ctrl.has_pose and self._ctrl.pos_y >= 8.85:
                self._pos_stable += 1
            else:
                self._pos_stable = 0
            if self._pos_stable >= self.POS_STABLE_NEEDED:
                turn_deg = math.degrees(normalize_angle(-1.57 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_CORRECT_9"
                self._pos_stable = 0
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, yaw correct {turn_deg:.1f}deg → -1.57")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[-self.WALK_SPEED, self._x_correct_vy(), 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # y=9处校正yaw到-1.57
        if self._state == "YAW_CORRECT_9":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw correct at y=9 done: {turned:.1f}deg")
                self._state = "BACKWALK2b"
                self._t0 = now
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        if self._state == "BACKWALK2b":
            if self._ctrl.has_pose and self._ctrl.pos_y >= 12.10:
                self._pos_stable += 1
            else:
                self._pos_stable = 0
            if self._pos_stable >= self.POS_STABLE_NEEDED:
                self._turn_target = -90.0           # 右转90°
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "TURN_RIGHT"
                self._pos_stable = 0
                self.get_logger().info(f"[S5] Reached y={self._ctrl.pos_y:.3f}, turning right 90°...")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[-self.WALK_SPEED, self._x_correct_vy(), 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "TURN_RIGHT":
            turned = self._turned_deg()
            target = self._turn_target
            done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Right turn done: {turned:.1f}deg, walking 3m...")
                self._state = "WALK2"
                self._t0 = now
                self._start_pos = (self._ctrl.pos_x, self._ctrl.pos_y) if self._ctrl.has_pose else None
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, -self.TURN_SPEED],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        # 右转后直走，防滑（roll条件y>=12.4）
        if self._state == "WALK2":
            if self._ctrl.has_pose and self._ctrl.pos_x <= 2.5:
                turn_deg = math.degrees(normalize_angle(-3.14 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_FIX"
                self.get_logger().info(
                    f"[S5] Reached x={self._ctrl.pos_x:.3f}, yaw fix {turn_deg:.1f}deg → -3.14")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[self._get_roll(), 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "YAW_FIX":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw fix done: {turned:.1f}deg, walking to x=2.0...")
                self._state = "WALK2b"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, self._y_correct_vy(12.4), wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        # x=2处校正yaw到-3.14 + y校正到12.4
        if self._state == "WALK2b":
            if self._ctrl.has_pose and self._ctrl.pos_x <= 2.0:
                turn_deg = math.degrees(normalize_angle(-3.14 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_FIX_X2"
                self.get_logger().info(
                    f"[S5] Reached x={self._ctrl.pos_x:.3f}, yaw fix {turn_deg:.1f}deg → -3.14")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[self._get_roll(), 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "YAW_FIX_X2":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw fix at x=2 done: {turned:.1f}deg")
                self._state = "WALK3"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, self._y_correct_vy(12.4), wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        # 继续直走到x<=-0.25，防滑
        if self._state == "WALK3":
            if self._ctrl.has_pose and self._ctrl.pos_x <= -0.25:
                self._turn_target = -90.0
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "TURN_RIGHT_90"
                self.get_logger().info(
                    f"[S5] Reached x={self._ctrl.pos_x:.3f}, turning right 90°...")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[self._get_roll(), 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 右转90°
        if self._state == "TURN_RIGHT_90":
            turned = self._turned_deg()
            target = self._turn_target
            done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S5] Right turn 90 done: {turned:.1f}deg, final walk...")
                self._state = "WALK4"
                self._t0 = now
                self._start_pos = (self._ctrl.pos_x, self._ctrl.pos_y) if self._ctrl.has_pose else None
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, -self.TURN_SPEED],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 直走到y>=12.5，防滑
        if self._state == "WALK4":
            if self._ctrl.has_pose and self._ctrl.pos_y >= 12.5:
                turn_deg = math.degrees(normalize_angle(1.57 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_FIX_2"
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, yaw fix {turn_deg:.1f}deg → 1.57")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 校正yaw到1.57 + x校正到-0.35
        if self._state == "YAW_FIX_2":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S5] Yaw fix 2 done: {turned:.1f}deg, walking to y=15.2...")
                self._state = "WALK5"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, self._x_correct_vy(target=-0.45), wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 直走，防滑
        if self._state == "WALK5":
            if self._ctrl.has_pose and self._ctrl.pos_y >= 13.0:
                turn_deg = math.degrees(normalize_angle(1.57 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_FIX_13"
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, yaw fix {turn_deg:.1f}deg → 1.57")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # y=13处校正yaw到1.57 + x校正到-0.35
        if self._state == "YAW_FIX_13":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw fix at y=13 done: {turned:.1f}deg")
                self._state = "WALK5b"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, self._x_correct_vy(target=-0.45), wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "WALK5b":
            if self._ctrl.has_pose and self._ctrl.pos_y >= 15.1:
                self._turn_target = -90.0
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "TURN_RIGHT_3"
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, turning right 90°...")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "TURN_RIGHT_3":
            turned = self._turned_deg()
            target = self._turn_target
            done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S5] Right turn 3 done: {turned:.1f}deg, walking to x<=3.1...")
                self._state = "WALK6"
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, -self.TURN_SPEED],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        # 往回走，防滑
        if self._state == "WALK6":
            if self._ctrl.has_pose and self._ctrl.pos_x >= 2.8:
                self._turn_target = -90.0
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "TURN_RIGHT_4"
                self.get_logger().info(
                    f"[S5] Reached x={self._ctrl.pos_x:.3f}, turning right 90°...")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, self._anti_slide_vy(), 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "TURN_RIGHT_4":
            turned = self._turned_deg()
            target = self._turn_target
            done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S5] Right turn 4 done: {turned:.1f}deg, walking to y<=13.4...")
                self._state = "WALK7"
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, -self.TURN_SPEED],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        # 最后直走，y递减
        if self._state == "WALK7":
            if self._ctrl.has_pose and self._ctrl.pos_y <= 14.5:
                turn_deg = math.degrees(normalize_angle(-1.57 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_FIX_145"
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, yaw fix {turn_deg:.1f}deg → -1.57")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "YAW_FIX_145":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw fix at y=14.5 done: {turned:.1f}deg")
                self._state = "WALK7b"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "WALK7b":
            if self._ctrl.has_pose and self._ctrl.pos_y <= 14.0:
                turn_deg = math.degrees(normalize_angle(-1.57 - self._ctrl.yaw))
                self._turn_target = turn_deg
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "YAW_FIX_14"
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, yaw fix {turn_deg:.1f}deg → -1.57")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "YAW_FIX_14":
            turned = self._turned_deg()
            target = self._turn_target
            if target > 0:
                done = turned is not None and turned >= target - self.TURN_TOL_DEG
            else:
                done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(f"[S5] Yaw fix at y=14 done: {turned:.1f}deg")
                self._state = "WALK7c"
                return
            wz = self.YAW_CORRECT_SPEED if target > 0 else -self.YAW_CORRECT_SPEED
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "WALK7c":
            if self._ctrl.has_pose and self._ctrl.pos_y <= 13.65:
                self._turn_target = -90.0
                self._turn_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else None
                self._state = "TURN_RIGHT_END"
                self.get_logger().info(
                    f"[S5] Reached y={self._ctrl.pos_y:.3f}, turning right 90...")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.05, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        if self._state == "TURN_RIGHT_END":
            turned = self._turned_deg()
            target = self._turn_target
            done = turned is not None and turned <= target + self.TURN_TOL_DEG
            if done:
                self.get_logger().info(
                    f"[S5] Final right turn done: {turned:.1f}deg -> WALK_8CM")
                self._state = "WALK_8CM"
                self._t0 = now
                self._walk_8cm_start_x = self._ctrl.pos_x if self._ctrl.has_pose else 0.0
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, -self.TURN_SPEED],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.CLIMB_STEP, self.CLIMB_STEP],
            )
            return

        # 前进8cm后跳
        if self._state == "WALK_8CM":
            if self._ctrl.has_pose:
                dx = self._ctrl.pos_x - self._walk_8cm_start_x
            else:
                dx = 0.0
            if dx >= 0.08:
                self.get_logger().info(f"[S5] Walk 8cm done (dx={dx:.3f}) -> JUMP_START")
                self._state = "JUMP_START"
                self._t0 = now
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # Final jump
        if self._state == "JUMP_START":
            self._jump_start_x = self._ctrl.pos_x
            self._jump_start_y = self._ctrl.pos_y
            self._jump_start_yaw = self._ctrl.yaw if self._ctrl.has_yaw else 0.0
            self._jump_start_t = now
            self._state = "JUMP_FWD"
            self.get_logger().info("[S5] Final jump start")
            return

        if self._state == "JUMP_FWD":
            elapsed = now - self._jump_start_t
            dx = self._ctrl.pos_x - self._jump_start_x
            dy = self._ctrl.pos_y - self._jump_start_y
            progress = dx * math.cos(self._jump_start_yaw) + dy * math.sin(self._jump_start_yaw)
            done = (progress >= 1.0) or (elapsed >= 6.0)
            if done:
                self._state = "JUMP_SETTLE"
                self._t0 = now
                self.get_logger().info(f"[S5] Jump done (progress={progress:.3f}) -> JUMP_SETTLE")
                return
            self._ctrl.send_cmd(
                MODE_JUMP, gait_id=GAIT_JUMP,
                vel_des=[0.70, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[0.06, 0.06],
            )
            return

        if self._state == "JUMP_SETTLE":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[0.06, 0.06],
            )
            if now - self._t0 >= 1.6:
                self.get_logger().info("[S5] Jump settle done -> DONE")
                self._ctrl.stop()
                self._state = "DONE"
            return

        if self._state == "DONE":
            self._ctrl._running = False
            self.get_logger().info("[S5] Shutdown complete")
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Stage5Walk()
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
