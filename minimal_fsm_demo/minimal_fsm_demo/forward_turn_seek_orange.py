#!/usr/bin/env python3
"""
Forward 1m -> slow left turn 90 deg.
If orange ball is detected during the route, switch to seek mode and move toward it.
"""

import enum
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    import lcm
except Exception:
    lcm = None

try:
    from minimal_fsm_demo.lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
except Exception:
    robot_control_cmd_lcmt = None

try:
    from minimal_fsm_demo.lcm_types.localization_lcmt import localization_lcmt
except Exception:
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


class State(enum.Enum):
    INIT = "INIT"
    STAND_UP = "STAND_UP"
    FORWARD_1M = "FORWARD_1M"
    TURN_LEFT_90 = "TURN_LEFT_90"
    FORWARD_AFTER_TURN = "FORWARD_AFTER_TURN"
    SEEK_ORANGE = "SEEK_ORANGE"
    DONE = "DONE"


class RobotCtrl:
    def __init__(self, node):
        self._node = node
        self._lcm_cmd = None
        self._lcm_rpy = None
        self._lock = threading.Lock()
        self._cmd_msg = robot_control_cmd_lcmt()
        self._running = True
        self._life_count = 0
        self.yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.has_pose = False

        if lcm is None or robot_control_cmd_lcmt is None:
            self._node.get_logger().error("LCM unavailable!")
            return

        self._lcm_cmd = lcm.LCM(LCM_CMD_URL)
        if localization_lcmt is not None:
            self._lcm_rpy = lcm.LCM(LCM_RPY_URL)
            self._lcm_rpy.subscribe(RPY_CHANNEL, self._on_rpy)
            threading.Thread(target=self._rpy_loop, daemon=True).start()

        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _on_rpy(self, channel, data):
        msg = localization_lcmt().decode(data)
        self.yaw = msg.rpy[2]
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


class ForwardTurnSeekOrange(Node):
    WALK_SPEED = 0.25
    TURN_SPEED = 0.20
    STEP_HEIGHT = 0.06
    FORWARD_DIST = 1.0
    STAND_DUR = 1.5
    SEEK_TIMEOUT = 12.0
    FORWARD_AFTER_SPEED = 0.18

    def __init__(self):
        super().__init__("forward_turn_seek_orange")
        self.state = State.INIT
        self._t0 = 0.0
        self._ctrl = RobotCtrl(self)
        self._start_xy = None
        self._turn_start_yaw = None

        self._orange_seen = False
        self._last_ball_ts = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._target_yaw = None

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(String, "/ball_color", self._ball_cb, qos)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[FTSO] Ready")

    def _ball_cb(self, msg: String):
        if msg.data.strip().lower() == "orange":
            self._orange_seen = True
            self._last_ball_ts = time.time()

    def _get_cam_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform("base_link", "rgb_camera_link", rclpy.time.Time())
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny, cosy)
        except Exception:
            return None

    def _switch_seek(self):
        self.state = State.SEEK_ORANGE
        self._t0 = time.time()
        self.get_logger().info("[FTSO] Orange detected -> SEEK_ORANGE")

    def control_loop(self):
        now = time.time()

        if self.state in (State.FORWARD_1M, State.TURN_LEFT_90) and self._orange_seen:
            self._switch_seek()
            return

        if self.state == State.INIT:
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            self.state = State.STAND_UP
            self._t0 = now
            self.get_logger().info("[FTSO] Recovery stand")
            return

        if self.state == State.STAND_UP:
            if now - self._t0 >= self.STAND_DUR:
                self.state = State.FORWARD_1M
                self._t0 = now
                if self._ctrl.has_pose:
                    self._start_xy = (self._ctrl.pos_x, self._ctrl.pos_y)
                self.get_logger().info("[FTSO] Forward 1m")
            return

        if self.state == State.FORWARD_1M:
            if self._start_xy and self._ctrl.has_pose:
                dx = self._ctrl.pos_x - self._start_xy[0]
                dy = self._ctrl.pos_y - self._start_xy[1]
                dist = math.hypot(dx, dy)
                if dist >= self.FORWARD_DIST:
                    self.state = State.TURN_LEFT_90
                    self._turn_start_yaw = self._ctrl.yaw
                    self.get_logger().info("[FTSO] -> TURN_LEFT_90")
                    return
            else:
                # fallback by time if no pose
                if now - self._t0 > self.FORWARD_DIST / self.WALK_SPEED * 1.5:
                    self.state = State.TURN_LEFT_90
                    self._turn_start_yaw = self._ctrl.yaw
                    self.get_logger().warn("[FTSO] No pose fallback -> TURN_LEFT_90")
                    return

            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.TURN_LEFT_90:
            if self._turn_start_yaw is None:
                self._turn_start_yaw = self._ctrl.yaw
            yaw_delta = normalize_angle(self._ctrl.yaw - self._turn_start_yaw)
            if yaw_delta >= math.radians(90.0):
                self.state = State.FORWARD_AFTER_TURN
                self.get_logger().info("[FTSO] Turn done -> FORWARD_AFTER_TURN")
                return

            # slow left turn
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, self.TURN_SPEED],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.FORWARD_AFTER_TURN:
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[self.FORWARD_AFTER_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.SEEK_ORANGE:
            if now - self._t0 > self.SEEK_TIMEOUT:
                self.get_logger().warn("[FTSO] SEEK timeout -> DONE")
                self.state = State.DONE
                return

            cam_yaw = self._get_cam_yaw()
            if cam_yaw is None:
                # keep walking slowly forward if transform is unavailable
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                    vel_des=[0.14, 0.0, 0.0],
                    step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
                return

            # Proportional steering to face target
            yaw_rate = max(-0.35, min(0.35, 1.2 * cam_yaw))
            forward = 0.14 if abs(cam_yaw) < 0.55 else 0.07
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[forward, 0.0, yaw_rate],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.DONE:
            self._ctrl.send_cmd(MODE_LOCOMOTION, gait_id=GAIT_STAND, vel_des=[0.0, 0.0, 0.0])


def main(args=None):
    rclpy.init(args=args)
    node = ForwardTurnSeekOrange()
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
