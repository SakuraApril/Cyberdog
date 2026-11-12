#!/usr/bin/env python3
"""
Stage6: Multi-waypoint navigation with jump start and PID rotation.
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


# PID rotation constants
PID_KP = 0.8
PID_KI = 0.02
PID_KD = 0.3
PID_OUTPUT_LIMIT = 1.0
PID_INTEGRAL_LIMIT = 0.3
PID_FILTER_ALPHA = 0.2
PID_TOL_RAD = 0.035       # ~2°
PID_STOP_RATE = 0.08      # rad/s


class PIDControllerRad:
    def __init__(self):
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._filtered_d = 0.0

    def calculate(self, target, measurement, dt):
        error = normalize_angle(target - measurement)
        p = PID_KP * error
        if abs(error) < 0.087:  # 5°
            self._integral += error * dt
            self._integral = max(-PID_INTEGRAL_LIMIT,
                                 min(PID_INTEGRAL_LIMIT, self._integral))
        i = PID_KI * self._integral
        if self._prev_meas is not None:
            raw_d = (measurement - self._prev_meas) / dt
            self._filtered_d = (PID_FILTER_ALPHA * raw_d +
                                (1 - PID_FILTER_ALPHA) * self._filtered_d)
        d = -PID_KD * self._filtered_d
        self._prev_meas = measurement
        output = p + i + d
        return max(-PID_OUTPUT_LIMIT, min(PID_OUTPUT_LIMIT, output))


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
                f"[Ctrl] Cal pos: offset=({self._cal_x:.3f}, {self._cal_y:.3f})")

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
    # Hardcoded Gazebo start coordinates (stable reference for waypoints)
    _REF_START_X = 4.513
    _REF_START_Y = 23.254
    # Offsets from origin will be used at runtime
    _DX_WP1 = 0.35 - 2.5          # -2.15
    _DY_WP1 = 13.50052 - 13.50052 # 0
    _DX_WP2 = 0.2 - 2.5           # -2.3
    _DY_WP2 = 14.90052 - 13.50052 # +1.4
    _DX_A = 2.0 - 2.5             # -0.5
    _DY_A = 13.8 - 13.50052       # +0.29948
    _DX_B = 2.4 - 2.5             # -0.1
    _DY_B = 13.0 - 13.50052       # -0.50052
    _DX_C = 4.0 - 2.5             # +1.5
    _DY_C = 12.95 - 13.50052      # -0.55052
    _X_318_OFFSET = 3.18 - 2.5    # +0.68
    _Y_12_9_OFFSET = 12.9 - 13.50052  # -0.60052
    WALK_SPEED = 0.30
    WALK_STEP = 0.08
    TURN_SPEED = 0.35
    TURN_TOL_DEG = 4.0
    STAND_DUR = 8.0
    TURN1_YAW = 1.56
    TURN2_YAW = -0.5
    YA_A = -0.5
    YA_B = -1.0
    YA_C = 0.0
    WP_TOL = 0.05

    def __init__(self):
        super().__init__("stage6_nav")
        self._ctrl = RobotCtrl(self)
        self._state = "WAIT_ORIGIN"
        self._t0 = time.time()
        self._turn_target = 0.0
        self._turn_start_yaw = None
        self._origin_x = None
        self._origin_y = None
        self._origin_set = False
        self._last_walk_debug_t = 0.0
        # Waypoints (computed after origin is set)
        self._wp1 = (0.0, 0.0)
        self._wp2 = (0.0, 0.0)
        self._wp_a = (0.0, 0.0, -0.5)
        self._wp_b = (0.0, 0.0, -1.0)
        self._wp_c = (0.0, 0.0, 0.0)
        self._x318_target = 0.0
        self._y129_target = 0.0

        # PID rotation state
        self._pid = PIDControllerRad()
        self._pid_active = False
        self._pid_target = 0.0
        self._pid_prev_yaw = 0.0
        self._pid_filtered_rate = 0.0
        self._pid_t0 = 0.0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("[S6] Waiting for LCM pose to set origin...")

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

    def _pid_rotate_step(self, relative_rad):
        """Non-blocking PID rotation. Returns True when stable."""
        if not self._ctrl.has_yaw:
            return False
        if not self._pid_active:
            self._pid.reset()
            current = normalize_angle(self._ctrl.yaw)
            self._pid_target = normalize_angle(current + relative_rad)
            self._pid_prev_yaw = current
            self._pid_filtered_rate = 0.0
            self._pid_t0 = now = time.time()
            self._pid_active = True
            self.get_logger().info(
                f"[S6] PID rotate: yaw={math.degrees(current):.1f}° "
                f"by {math.degrees(relative_rad):.1f}° → {math.degrees(self._pid_target):.1f}°")

        dt = 0.05
        current = normalize_angle(self._ctrl.yaw)
        error = normalize_angle(self._pid_target - current)
        raw_rate = normalize_angle(current - self._pid_prev_yaw) / dt
        self._pid_filtered_rate = 0.25 * raw_rate + 0.75 * self._pid_filtered_rate
        self._pid_prev_yaw = current

        if abs(error) < PID_TOL_RAD and abs(self._pid_filtered_rate) < PID_STOP_RATE:
            elapsed = time.time() - self._pid_t0
            self.get_logger().info(
                f"[S6] PID rotate done: yaw={math.degrees(current):.1f}° "
                f"err={math.degrees(error):.2f}° "
                f"rate={math.degrees(self._pid_filtered_rate):.2f}°/s "
                f"time={elapsed:.1f}s")
            self._pid_active = False
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_STAND,
                vel_des=[0.0, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return True

        yaw_rate = self._pid.calculate(self._pid_target, current, dt)
        limit = PID_OUTPUT_LIMIT if abs(error) > 0.14 else 0.25
        yaw_rate = max(-limit, min(limit, yaw_rate))
        self._ctrl.send_cmd(
            MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
            vel_des=[0.0, 0.0, yaw_rate],
            rpy_des=[0.0, 0.0, 0.0],
            step_height=[self.WALK_STEP, self.WALK_STEP],
        )
        return False

    def control_loop(self):
        now = time.time()

        # 0) WAIT_ORIGIN: calibrate LCM to hardcoded Gazebo reference
        if self._state == "WAIT_ORIGIN":
            if self._ctrl.has_pose:
                self._ctrl.set_calibration(self._REF_START_X, self._REF_START_Y)
                self._origin_x = self._REF_START_X
                self._origin_y = self._REF_START_Y
                self._origin_set = True
                self.get_logger().info(
                    f"[S6] Hardcoded Gazebo origin: ({self._origin_x:.4f},{self._origin_y:.4f})")
                # Compute all waypoints from origin
                self._wp1 = (self._origin_x + self._DX_WP1, self._origin_y + self._DY_WP1)
                self._wp2 = (self._origin_x + self._DX_WP2, self._origin_y + self._DY_WP2)
                self._wp_a = (self._origin_x + self._DX_A, self._origin_y + self._DY_A, self.YA_A)
                self._wp_b = (self._origin_x + self._DX_B, self._origin_y + self._DY_B, self.YA_B)
                self._wp_c = (self._origin_x + self._DX_C, self._origin_y + self._DY_C, self.YA_C)
                self._x318_target = self._origin_x + self._X_318_OFFSET
                self._y129_target = self._origin_y + self._Y_12_9_OFFSET
                self.get_logger().info(
                    f"[S6] Origin=({self._origin_x:.3f},{self._origin_y:.3f})")
                self.get_logger().info(
                    f"[S6] WP1=({self._wp1[0]:.3f},{self._wp1[1]:.3f}) "
                    f"WP2=({self._wp2[0]:.3f},{self._wp2[1]:.3f})")
                self.get_logger().info(
                    f"[S6] A=({self._wp_a[0]:.3f},{self._wp_a[1]:.3f}) "
                    f"B=({self._wp_b[0]:.3f},{self._wp_b[1]:.3f}) "
                    f"C=({self._wp_c[0]:.3f},{self._wp_c[1]:.3f})")
                self._state = "INIT"
            return

        # 1) INIT → STAND
        if self._state == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[0.06, 0.06],
            )
            self._state = "STAND"
            self._t0 = now
            self.get_logger().info("[S6] Recovery stand")
            return

        # 2) STAND → WALK_TO_WP1
        if self._state == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                self._state = "WALK_TO_WP1"
                self._t0 = now
                self.get_logger().info("[S6] Stand done -> WALK_TO_WP1")
            return

        # 3) WALK_TO_WP1 → TURN1
        # (was step 6, renumbered after removing jump)
        if self._state == "WALK_TO_WP1":
            wx, wy = self._wp1
            if self._ctrl.has_pose and self._ctrl.pos_x <= wx:
                self.get_logger().info(
                    f"[S6] Reached WP1 x={self._ctrl.pos_x:.3f} -> TURN1")
                self._turn_relative_rad = normalize_angle(self.TURN1_YAW - self._ctrl.yaw) if self._ctrl.has_yaw else math.radians(-90.0)
                self._state = "TURN1"
                self._t0 = now
                return
            if self._ctrl.has_pose and self._ctrl.has_yaw:
                target = self._heading_to(wx, wy)
                err = normalize_angle(target - self._ctrl.yaw)
                wz = max(-0.3, min(0.3, err * 1.5))
            else:
                wz = 0.0
            if now - self._last_walk_debug_t >= 1.0 and self._ctrl.has_pose:
                self._last_walk_debug_t = now
                self.get_logger().info(
                    f"[S6] WALK_TO_WP1: x={self._ctrl.pos_x:.3f} "
                    f"y={self._ctrl.pos_y:.3f} yaw={self._ctrl.yaw:.3f}")
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, wz],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 7) TURN1 — PID rotate to TURN1_YAW
        if self._state == "TURN1":
            if self._pid_rotate_step(self._turn_relative_rad):
                self.get_logger().info("[S6] Turn1 done -> WALK_TO_WP2")
                self._state = "WALK_TO_WP2"
                self._t0 = now
            return

        # 8) WALK_TO_WP2 → TURN2
        if self._state == "WALK_TO_WP2":
            wx, wy = self._wp2
            dist = self._distance_to(wx, wy)
            if dist is not None and dist <= 0.2:
                self.get_logger().info(
                    f"[S6] Reached WP2 (dist={dist:.3f}) -> TURN2")
                self._turn_relative_rad = normalize_angle(self.TURN2_YAW - self._ctrl.yaw) if self._ctrl.has_yaw else math.radians(-135.0)
                # Force right turn
                if self._turn_relative_rad > 0:
                    self._turn_relative_rad -= 2 * math.pi
                self._state = "TURN2"
                self._t0 = now
                return
            if self._ctrl.has_pose and self._ctrl.has_yaw:
                target = self._heading_to(wx, wy)
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

        # 9) TURN2 — PID rotate to TURN2_YAW
        if self._state == "TURN2":
            if self._pid_rotate_step(self._turn_relative_rad):
                self.get_logger().info("[S6] Turn2 done -> WALK_TO_A")
                self._state = "WALK_TO_A"
            return

        # 10) WALK_TO_A: walk to WP_A
        if self._state == "WALK_TO_A":
            tx, ty, tyaw = self._wp_a
            dist = self._distance_to(tx, ty)
            if dist is not None and dist <= self.WP_TOL:
                self.get_logger().info(f"[S6] Reached A ({tx},{ty}) dist={dist:.3f} -> TURN_AT_A")
                self._state = "TURN_AT_A"
                return
            if now - self._last_walk_debug_t >= 1.0 and dist is not None:
                self._last_walk_debug_t = now
                self.get_logger().info(
                    f"[S6] WALK_TO_A: dist={dist:.3f} "
                    f"pose=({self._ctrl.pos_x:.3f},{self._ctrl.pos_y:.3f})")
            if self._ctrl.has_pose and self._ctrl.has_yaw:
                target = self._heading_to(tx, ty)
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

        # 11) TURN_AT_A: turn to target yaw
        if self._state == "TURN_AT_A":
            _, _, tyaw = self._wp_a
            if self._ctrl.has_yaw:
                rel = normalize_angle(tyaw - self._ctrl.yaw)
                if rel > 0:
                    rel -= 2 * math.pi
            else:
                rel = -math.pi / 2
            if self._pid_rotate_step(rel):
                self.get_logger().info("[S6] Turn at A done -> WALK_TO_B")
                self._state = "WALK_TO_B"
            return

        # 12) WALK_TO_B: walk to WP_B
        if self._state == "WALK_TO_B":
            tx, ty, tyaw = self._wp_b
            if self._ctrl.has_pose:
                if self._ctrl.pos_x >= tx or self._ctrl.pos_y <= ty:
                    self.get_logger().info(
                        f"[S6] Reached B ({tx},{ty}) "
                        f"pose=({self._ctrl.pos_x:.3f},{self._ctrl.pos_y:.3f}) -> TURN_AT_B")
                    self._state = "TURN_AT_B"
                    return
            if now - self._last_walk_debug_t >= 1.0:
                self._last_walk_debug_t = now
                self.get_logger().info(
                    f"[S6] WALK_TO_B: pose=({self._ctrl.pos_x:.3f},{self._ctrl.pos_y:.3f})")
            if self._ctrl.has_pose and self._ctrl.has_yaw:
                target = self._heading_to(tx, ty)
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

        # 13) TURN_AT_B: turn to -π/2
        if self._state == "TURN_AT_B":
            if self._ctrl.has_yaw:
                rel = normalize_angle(-math.pi / 2 - self._ctrl.yaw)
                if rel > 0:
                    rel -= 2 * math.pi
            else:
                rel = -math.pi / 2
            if self._pid_rotate_step(rel):
                self.get_logger().info("[S6] Turn at B done (yaw=-π/2) -> WALK_3S")
                self._state = "WALK_3S_AFTER_B"
                self._t0 = now
            return

        # 14) WALK_5S_AFTER_B: straight walk 5s (no heading correction)
        if self._state == "WALK_3S_AFTER_B":
            if now - self._t0 >= 5.0:
                self.get_logger().info("[S6] Walk 5s done -> TURN_TO_0")
                self._state = "TURN_TO_0_AFTER_B"
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 15) TURN_TO_0_AFTER_B: turn to yaw=0
        if self._state == "TURN_TO_0_AFTER_B":
            if self._ctrl.has_yaw:
                rel = normalize_angle(0.0 - self._ctrl.yaw)
                if rel > 0:
                    rel -= 2 * math.pi
            else:
                rel = -math.pi / 2
            if self._pid_rotate_step(rel):
                self.get_logger().info("[S6] Turn to 0 done -> CORRECT_Y12_9")
                self._state = "CORRECT_Y12_9"
            return

        # 16) CORRECT_Y12_9: correct y
        if self._state == "CORRECT_Y12_9":
            target_y = self._y129_target
            if self._ctrl.has_pose and self._ctrl.pos_y <= target_y:
                self.get_logger().info(
                    f"[S6] Y corrected y={self._ctrl.pos_y:.3f} -> WALK_TO_C")
                self._state = "WALK_TO_C"
                return
            vy = max(-0.12, min(0.12, (target_y - self._ctrl.pos_y) * 1.0)) if self._ctrl.has_pose else -0.05
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[0.0, vy, 0.0],
                rpy_des=[0.0, 0.0, 0.0],
                step_height=[self.WALK_STEP, self.WALK_STEP],
            )
            return

        # 18) WALK_TO_C: walk to WP_C
        if self._state == "WALK_TO_C":
            tx, ty, tyaw = self._wp_c
            dist = self._distance_to(tx, ty)
            if dist is not None and (dist <= self.WP_TOL or self._ctrl.pos_x >= 3.1):
                self.get_logger().info(f"[S6] Reached C ({tx},{ty}) dist={dist:.3f} -> SIT_DOWN")
                self._state = "SIT_DOWN"
                self._t0 = now
                return
            if now - self._last_walk_debug_t >= 1.0 and dist is not None:
                self._last_walk_debug_t = now
                self.get_logger().info(
                    f"[S6] WALK_TO_C: dist={dist:.3f} "
                    f"pose=({self._ctrl.pos_x:.3f},{self._ctrl.pos_y:.3f})")
            if self._ctrl.has_pose and self._ctrl.has_yaw:
                target = self._heading_to(tx, ty)
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

        # SIT_DOWN → DONE
        if self._state == "SIT_DOWN":
            self._ctrl.send_cmd(MODE_PURE_DAMPER, duration=3000)
            self.get_logger().info("[S6] Damper (sit down)...")
            self._state = "WAIT_SIT"
            self._t0 = now
            return

        if self._state == "WAIT_SIT":
            if now - self._t0 >= 3.0:
                self._ctrl.stop()
                self.get_logger().info("[S6] Completed")
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
