#!/usr/bin/env python3
"""
直走5m + 检测前方黑色物体（可乐瓶）
走完后用摄像头检测，识别到黑色物体则语音播报
"""
import enum
import os
import time
import threading

# 信号文件路径（Windows端读取这个文件来播报）
SIGNAL_FILE = "/tmp/cyberdog_say.txt"

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import lcm
except Exception:
    lcm = None

try:
    from minimal_fsm_demo.lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
except Exception:
    robot_control_cmd_lcmt = None

try:
    from minimal_fsm_demo.audio_announcer import init_audio, say
except Exception:
    init_audio = None
    say = None

LCM_CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
CMD_CHANNEL = "robot_control_cmd"
MODE_PURE_DAMPER = 7
MODE_LOCOMOTION = 11
MODE_RECOVERY_STAND = 12
GAIT_TROT = 26
CONTACT_ALL = 15

# 黑色物体HSV范围（低亮度）
BLACK_LOWER = np.array([0, 0, 0])
BLACK_UPPER = np.array([180, 255, 60])


class State(enum.Enum):
    INIT = "INIT"
    STAND_UP = "STAND_UP"
    WALK = "WALK"
    DETECT = "DETECT"
    DONE = "DONE"


class RobotCtrl:
    def __init__(self):
        self._lcm_cmd = None
        self._lock = threading.Lock()
        self._cmd_msg = robot_control_cmd_lcmt()
        self._running = True
        self._life_count = 0
        if lcm and robot_control_cmd_lcmt:
            self._lcm_cmd = lcm.LCM(LCM_CMD_URL)
            threading.Thread(target=self._heartbeat, daemon=True).start()

    def send_cmd(self, mode, gait_id=0, vel_des=None, rpy_des=None,
                 step_height=None, duration=0):
        with self._lock:
            self._life_count = (self._life_count + 1) % 128
            msg = robot_control_cmd_lcmt()
            msg.mode = mode
            msg.gait_id = gait_id
            msg.contact = CONTACT_ALL
            msg.duration = duration
            msg.life_count = self._life_count
            if vel_des:
                for i, v in enumerate(vel_des[:3]):
                    msg.vel_des[i] = v
            if rpy_des:
                for i, v in enumerate(rpy_des[:3]):
                    msg.rpy_des[i] = v
            if step_height:
                for i, s in enumerate(step_height[:2]):
                    msg.step_height[i] = s
            self._cmd_msg = msg

    def stop(self):
        self.send_cmd(MODE_PURE_DAMPER, duration=4000)
        self._running = False

    def _heartbeat(self):
        while self._running:
            with self._lock:
                msg = self._cmd_msg
            if self._lcm_cmd and msg:
                self._lcm_cmd.publish(CMD_CHANNEL, msg.encode())
            time.sleep(0.1)


class WalkStraight(Node):
    WALK_SPEED = 0.3
    WALK_DIST = 5.0
    STEP_HEIGHT = 0.07   # 触发C++降低重心0.04m
    STAND_DUR = 1.0

    def __init__(self):
        super().__init__("walk_straight")
        self.state = State.INIT
        self._t0 = 0.0
        self._ctrl = RobotCtrl()

        # Vision
        self.bridge = CvBridge()
        self._frame = None
        self._frame_lock = threading.Lock()
        self._detected_black = False

        # Camera subscription
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/rgb_camera/image_raw', self._on_image, qos)

        # Audio
        if init_audio:
            init_audio(mode="sim")

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[WalkStraight] Ready")

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self._frame = frame
        except Exception:
            pass

    def _detect_black_object(self):
        """检测画面中央是否有黑色物体（可乐瓶）"""
        with self._frame_lock:
            frame = self._frame
        if frame is None:
            return False

        h, w = frame.shape[:2]
        # 只看画面中央下方区域（物体通常在前方地面）
        roi_y1 = int(h * 0.4)
        roi_y2 = int(h * 0.9)
        roi_x1 = int(w * 0.2)
        roi_x2 = int(w * 0.8)
        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLACK_LOWER, BLACK_UPPER)

        # 形态学去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        black_pixels = cv2.countNonZero(mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        ratio = black_pixels / total_pixels

        # 黑色像素占比 > 3% 认为有黑色物体
        self.get_logger().info(f"[Detect] Black ratio: {ratio:.3f} ({black_pixels}px)")
        return ratio > 0.03

    def _write_signal(self, text):
        """写信号文件供Windows端播报"""
        try:
            with open(SIGNAL_FILE, 'w', encoding='utf-8') as f:
                f.write(text)
        except Exception:
            pass

    def control_loop(self):
        now = time.time()

        if self.state == State.INIT:
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, gait_id=0,
                step_height=[0.06, 0.06], duration=5000)
            self.state = State.STAND_UP
            self._t0 = now
            self.get_logger().info("[M] Stand up...")
            if say: say("开始直行")
            return

        if self.state == State.STAND_UP:
            if now - self._t0 >= self.STAND_DUR:
                self.state = State.WALK
                self._t0 = now
                self.get_logger().info(f"[M] Walking {self.WALK_DIST}m...")
            return

        if self.state == State.WALK:
            walk_dur = self.WALK_DIST / self.WALK_SPEED
            elapsed = now - self._t0
            if elapsed >= walk_dur:
                self._ctrl.stop()
                self.state = State.DETECT
                self._t0 = now
                self.get_logger().info("[M] Walk done, detecting objects...")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                rpy_des=[0.0, 0.4, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.DETECT:
            # 停下来检测前方物体
            if now - self._t0 < 0.5:
                return  # 等待画面稳定

            if self._detect_black_object():
                self.get_logger().info("[M] Detected: 可乐瓶!")
                if say: say("识别到可乐瓶")
                self._write_signal("识别到可乐瓶")
            else:
                self.get_logger().info("[M] No black object detected")
                if say: say("未识别到物体")
                self._write_signal("未识别到物体")

            self.state = State.DONE
            return

        if self.state == State.DONE:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WalkStraight()
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
