#!/usr/bin/env python3
"""
Stage8:
- Blind-drive near ball first.
- Lower head a bit for near-ground ball view.
- Keep visually chasing/pushing white ball.
- Reach final circle and lie down.
"""

import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

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


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


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

        self._cal_ok = False
        self._cal_x = 0.0
        self._cal_y = 0.0
        self._cal_yaw_ok = False
        self._cal_yaw = 0.0
        self._raw_yaw = 0.0

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
            self._raw_yaw = float(msg.rpy[2])
            if self._cal_yaw_ok:
                self.yaw = normalize_angle(self._raw_yaw + self._cal_yaw)
            else:
                self.yaw = self._raw_yaw
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
            if ref_yaw is not None and self.has_yaw:
                self._cal_yaw = normalize_angle(ref_yaw - self._raw_yaw)
                self._cal_yaw_ok = True
                self.yaw = normalize_angle(self._raw_yaw + self._cal_yaw)
            self._node.get_logger().info(
                f"[S8] Pose calibration: offset=({self._cal_x:.3f}, {self._cal_y:.3f}), "
                f"yaw_off={self._cal_yaw:.3f}")

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


class Stage8BallAndFinish(Node):
    # User-given coordinates
    START_X = 2.5
    START_Y = 13.50052
    START_YAW = 3.08662
    BALL_X = 0.4
    BALL_Y = 14.70052
    BALL_YAW = 3.08662

    GOAL_X = 3.2
    GOAL_Y = 12.90052
    GOAL_YAW = 3.08662

    STAND_DUR = 1.8
    STEP_H = 0.07
    HEAD_PITCH_DOWN = -0.18

    # Vision chase params
    SEARCH_WZ = 0.26
    CHASE_V_MAX = 0.42
    CHASE_V_MIN = 0.12
    STEER_GAIN = 1.8
    STEER_LIMIT = 0.90
    BALL_LOST_SEC = 0.8
    PUSH_AREA = 0.10
    PUSH_ALIGN_ERR = 0.16
    PUSH_V = 0.62

    # Navigate to final circle
    GOAL_POS_TOL = 0.22
    GOAL_YAW_TOL_DEG = 5.0
    CHASE_MAX_DUR = 60.0
    BLIND_BALL_TOL = 0.35
    BLIND_BALL_MAX_DUR = 22.0

    def __init__(self):
        super().__init__("stage8_ball_and_finish")
        self._ctrl = RobotCtrl(self)
        self._state = "INIT"
        self._t0 = time.time()
        self._chase_t0 = 0.0
        self._nav_t0 = 0.0
        self._calibrated = False

        # Vision state
        self._ball_seen = False
        self._ball_last_seen_t = 0.0
        self._ball_cx = 0.5
        self._ball_cy = 0.5
        self._ball_area = 0.0
        self._last_debug_t = 0.0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, "/rgb_camera/image_raw", self._on_image, qos)
        self.create_subscription(
            Image, "/rgb_camera/rgb_camera_sensor/image_raw", self._on_image, qos
        )

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("====================================")
        self.get_logger().info("[S8] Stage8 started")
        self.get_logger().info("[S8] White ball visual chase + kick")
        self.get_logger().info(
            f"[S8] Start=({self.START_X:.3f},{self.START_Y:.3f},{self.START_YAW:.3f})")
        self.get_logger().info(
            f"[S8] Ball =({self.BALL_X:.3f},{self.BALL_Y:.3f},{self.BALL_YAW:.3f})")
        self.get_logger().info(
            f"[S8] Goal =({self.GOAL_X:.3f},{self.GOAL_Y:.3f},{self.GOAL_YAW:.3f})")
        self.get_logger().info("====================================")

    def _on_image(self, msg: Image):
        try:
            if msg.encoding == "rgb8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
            elif msg.encoding == "rgba8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4
                )
                bgr = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgra8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4
                )
                bgr = img[:, :, :3]
            else:
                return
        except Exception:
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # White mask: low saturation, high value
        mask = cv2.inRange(hsv, (0, 0, 170), (180, 65, 255))
        # Ignore far area (upper image) to reduce false detections on bright props.
        h = msg.height
        mask[: int(0.25 * h), :] = 0
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = 0.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 120:
                continue
            peri = cv2.arcLength(c, True)
            if peri <= 1e-6:
                continue
            circularity = 4.0 * math.pi * area / (peri * peri)
            if circularity < 0.35:
                continue
            score = area * (0.5 + 0.5 * min(circularity, 1.0))
            if score > best_score:
                M = cv2.moments(c)
                if M["m00"] <= 1e-6:
                    continue
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                best = (cx, cy, area)
                best_score = score

        if best is not None:
            cx, cy, area = best
            self._ball_cx = float(cx / msg.width)
            self._ball_cy = float(cy / msg.height)
            self._ball_area = float(area / (msg.width * msg.height))
            self._ball_seen = True
            self._ball_last_seen_t = time.time()

    def _ball_fresh(self):
        if not self._ball_seen:
            return False
        return (time.time() - self._ball_last_seen_t) <= self.BALL_LOST_SEC

    def _drive_to_xy(self, tx, ty, pos_tol, vmax=0.28):
        dx = tx - self._ctrl.pos_x
        dy = ty - self._ctrl.pos_y
        dist = math.hypot(dx, dy)
        if dist <= pos_tol:
            return True

        desired = math.atan2(dy, dx)
        yaw_err = normalize_angle(desired - self._ctrl.yaw)
        wz = clamp(1.3 * yaw_err, -0.75, 0.75)

        if abs(yaw_err) > 1.0:
            vx = 0.0
        else:
            vx = vmax * (1.0 - abs(yaw_err) / 1.0)
            vx = clamp(vx, 0.08, vmax)

        self._ctrl.send_cmd(
            MODE_LOCOMOTION,
            gait_id=GAIT_TROT,
            vel_des=[vx, 0.0, wz],
            rpy_des=[0.0, self.HEAD_PITCH_DOWN, 0.0],
            step_height=[self.STEP_H, self.STEP_H],
        )
        return False

    def _drive_to_goal(self):
        return self._drive_to_xy(
            self.GOAL_X, self.GOAL_Y, self.GOAL_POS_TOL, vmax=0.28
        )

    def _align_goal_yaw(self):
        yaw_err = normalize_angle(self.GOAL_YAW - self._ctrl.yaw)
        if abs(math.degrees(yaw_err)) <= self.GOAL_YAW_TOL_DEG:
            return True
        wz = clamp(0.9 * yaw_err, -0.35, 0.35)
        self._ctrl.send_cmd(
            MODE_LOCOMOTION,
            gait_id=GAIT_TROT_SLOW,
            vel_des=[0.0, 0.0, wz],
            rpy_des=[0.0, self.HEAD_PITCH_DOWN, 0.0],
            step_height=[self.STEP_H, self.STEP_H],
        )
        return False

    def control_loop(self):
        now = time.time()

        if not self._calibrated and self._ctrl.has_pose and self._ctrl.has_yaw:
            self._ctrl.set_calibration(self.START_X, self.START_Y, self.START_YAW)
            self._calibrated = True

        if self._state == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND,
                contact=0,
                duration=5000,
                step_height=[0.06, 0.06],
            )
            self._state = "STAND"
            self._t0 = now
            self.get_logger().info("[S8] Recovery stand")
            return

        if self._state == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                self._state = "BLIND_TO_BALL"
                self._nav_t0 = now
                self.get_logger().info("[S8] Blind-drive near ball first")
            return

        if self._state == "BLIND_TO_BALL":
            if self._ctrl.has_pose and self._ctrl.has_yaw:
                reached_ball_zone = self._drive_to_xy(
                    self.BALL_X, self.BALL_Y, self.BLIND_BALL_TOL, vmax=0.30
                )
                if reached_ball_zone:
                    self._state = "CHASE_BALL"
                    self._chase_t0 = now
                    self.get_logger().info("[S8] Reached ball zone, switch to visual chase")
                    return
            else:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION,
                    gait_id=GAIT_TROT_SLOW,
                    vel_des=[0.12, 0.0, 0.0],
                    rpy_des=[0.0, self.HEAD_PITCH_DOWN, 0.0],
                    step_height=[self.STEP_H, self.STEP_H],
                )

            if now - self._nav_t0 > self.BLIND_BALL_MAX_DUR:
                self.get_logger().warn("[S8] Blind-drive timeout, switch to visual chase")
                self._state = "CHASE_BALL"
                self._chase_t0 = now
            return

        if self._state == "CHASE_BALL":
            goal_dist = None
            if self._ctrl.has_pose:
                goal_dist = math.hypot(
                    self.GOAL_X - self._ctrl.pos_x,
                    self.GOAL_Y - self._ctrl.pos_y,
                )
                if goal_dist <= self.GOAL_POS_TOL:
                    self._state = "ALIGN_YAW"
                    self.get_logger().info("[S8] Near goal circle, start yaw align")
                    return

            if self._ball_fresh():
                err = self._ball_cx - 0.5

                vx = clamp(
                    self.CHASE_V_MAX - 1.6 * self._ball_area,
                    self.CHASE_V_MIN,
                    self.CHASE_V_MAX,
                )
                if self._ball_area >= self.PUSH_AREA and abs(err) <= self.PUSH_ALIGN_ERR:
                    vx = max(vx, self.PUSH_V)
                # Image x: left->0, right->1; positive yaw is left turn.
                # So steering sign must be inverted: ball on right => turn right (wz < 0).
                wz = clamp(-self.STEER_GAIN * err, -self.STEER_LIMIT, self.STEER_LIMIT)
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION,
                    gait_id=GAIT_TROT,
                    vel_des=[vx, 0.0, wz],
                    rpy_des=[0.0, self.HEAD_PITCH_DOWN, 0.0],
                    step_height=[self.STEP_H, self.STEP_H],
                )
            else:
                # Search while creeping forward; default right-turn search.
                wz = -self.SEARCH_WZ
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION,
                    gait_id=GAIT_TROT_SLOW,
                    vel_des=[0.09, 0.0, wz],
                    rpy_des=[0.0, self.HEAD_PITCH_DOWN, 0.0],
                    step_height=[self.STEP_H, self.STEP_H],
                )

            if now - self._last_debug_t > 1.0:
                self._last_debug_t = now
                self.get_logger().info(
                    f"[S8] chase ball_seen={self._ball_fresh()} cx={self._ball_cx:.2f} area={self._ball_area:.3f} goal_dist={goal_dist}")

            if now - self._chase_t0 > self.CHASE_MAX_DUR:
                self.get_logger().warn("[S8] Chase timeout, go final align")
                self._state = "ALIGN_YAW"
            return

        if self._state == "ALIGN_YAW":
            if self._ctrl.has_yaw:
                if self._align_goal_yaw():
                    self._state = "LIE_DOWN"
                    self._t0 = now
                    self.get_logger().info("[S8] Yaw aligned, lying down")
            else:
                self._state = "LIE_DOWN"
                self._t0 = now
            return

        if self._state == "LIE_DOWN":
            self._ctrl.send_cmd(MODE_PURE_DAMPER, duration=4500)
            self._state = "DONE_WAIT"
            self._t0 = now
            return

        if self._state == "DONE_WAIT":
            if now - self._t0 >= 2.8:
                self.get_logger().info("[S8] Completed.")
                self._ctrl.stop()
                raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Stage8BallAndFinish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._ctrl.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
