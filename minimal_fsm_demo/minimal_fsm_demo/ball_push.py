#!/usr/bin/env python3
"""
Push 4 balls sequentially. Uses TF to get current position,
calculates angle/distance to each ball, turns and walks to push.

Balls:
  1: (0.8, 1.1)
  2: (2.1, 2.0)
  3: (3.2, 3.0)
  4: (-0.4, 3.8)
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

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

BALLS = [
    (0.8, 1.1),
    (2.1, 2.0),
    (3.2, 3.0),
    (-0.4, 3.8),
]

PUSH_DIST = 0.30  # push ball 30cm after reaching it


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


def normalize_angle(a):
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class BallPush(Node):

    WALK_SPEED = 0.3
    TURN_YAW = 1.3
    STEP_HEIGHT = 0.06
    STAND_DUR = 5.0
    TURN_TOL = 0.05  # ~3° tolerance
    POS_TOL = 0.15   # 15cm tolerance

    def __init__(self):
        super().__init__("ball_push")
        self._ctrl = RobotCtrl(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._ball_idx = 0
        self._phase = "INIT"  # INIT, STAND, TURN, WALK, PUSH, NEXT
        self._t0 = 0.0
        self._target_yaw = 0.0
        self._target_dist = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[BallPush] Ready")

    def _get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "vodom", "base_link", rclpy.time.Time())
            self._robot_x = t.transform.translation.x
            self._robot_y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._robot_yaw = math.atan2(siny, cosy)
            return True
        except Exception:
            return False

    def _calc_path(self, bx, by):
        """Calculate target yaw and distance from current pose to ball."""
        dx = bx - self._robot_x
        dy = by - self._robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        target_yaw = math.atan2(dy, dx)
        return target_yaw, dist

    def control_loop(self):
        now = time.time()

        # ── INIT ──
        if self._phase == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            self._phase = "STAND"
            self._t0 = now
            self.get_logger().info("[BP] Recovery stand")
            return

        # ── STAND ──
        if self._phase == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                if not self._get_pose():
                    return
                self._ball_idx = 0
                self._phase = "CALC"
            return

        # ── CALC ──
        if self._phase == "CALC":
            if not self._get_pose():
                return
            bx, by = BALLS[self._ball_idx]
            self._target_yaw, self._target_dist = self._calc_path(bx, by)
            yaw_err = normalize_angle(self._target_yaw - self._robot_yaw)
            self.get_logger().info(
                f"[BP] Ball {self._ball_idx+1} at ({bx},{by}), "
                f"dist={self._target_dist:.2f}m, "
                f"yaw_err={math.degrees(yaw_err):.1f}°")
            self._phase = "TURN"
            self._t0 = now
            return

        # ── TURN ──
        if self._phase == "TURN":
            if not self._get_pose():
                return
            yaw_err = normalize_angle(self._target_yaw - self._robot_yaw)
            if abs(yaw_err) < self.TURN_TOL:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_STAND,
                    vel_des=[0.0, 0.0, 0.0])
                self.get_logger().info(
                    f"[BP] Turn done (err={math.degrees(yaw_err):.1f}°)")
                self._phase = "WALK"
                self._t0 = now
                return
            # Turn toward target
            yaw_rate = self.TURN_YAW if yaw_err > 0 else -self.TURN_YAW
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[0.0, 0.0, yaw_rate],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        # ── WALK ──
        if self._phase == "WALK":
            if not self._get_pose():
                return
            bx, by = BALLS[self._ball_idx]
            _, dist = self._calc_path(bx, by)
            if dist <= self.POS_TOL:
                self.get_logger().info(
                    f"[BP] Reached ball {self._ball_idx+1} (dist={dist:.2f}m)")
                self._phase = "PUSH"
                self._t0 = now
                return
            # Walk forward toward ball
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        # ── PUSH ──
        if self._phase == "PUSH":
            push_dur = PUSH_DIST / self.WALK_SPEED
            if now - self._t0 >= push_dur:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_STAND,
                    vel_des=[0.0, 0.0, 0.0])
                self.get_logger().info(
                    f"[BP] Ball {self._ball_idx+1} pushed!")
                self._ball_idx += 1
                if self._ball_idx >= len(BALLS):
                    self._ctrl.stop()
                    self.get_logger().info("[BP] DONE! All 4 balls pushed.")
                    self._phase = "DONE"
                else:
                    self._phase = "CALC"
                    self._t0 = now
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self._phase == "DONE":
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BallPush()
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
