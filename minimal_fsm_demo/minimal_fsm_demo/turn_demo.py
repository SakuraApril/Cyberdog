#!/usr/bin/env python3
"""
TF-based closed-loop rotation demo.

Uses TF (vodom -> base_link) to read current yaw,
then rotates to target angles with P-control.
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
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ── Target angles (degrees) ──
TARGET_ANGLES_DEG = [90.0]  # change as needed, e.g. [90, 180, 270]


class TurnDemo(Node):

    STEP_HEIGHT = 0.06
    STAND_DUR = 5.0
    MAX_YAW_RATE = 1.3       # rad/s
    TURN_TOL = 0.05           # ~3 deg tolerance
    Kp = 2.0                  # proportional gain

    def __init__(self):
        super().__init__("turn_demo")
        self._ctrl = RobotCtrl(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._phase = "INIT"   # INIT, STAND, CALC, TURN, NEXT, DONE
        self._t0 = 0.0
        self._target_idx = 0
        self._target_yaw = 0.0
        self._start_yaw = None

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"[TurnDemo] Targets: {[f'{a}°' for a in TARGET_ANGLES_DEG]}")

    def _get_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "vodom", "base_link", rclpy.time.Time())
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny, cosy)
        except Exception:
            return None

    def control_loop(self):
        now = time.time()

        # ── INIT: recovery stand ──
        if self._phase == "INIT":
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            self._phase = "STAND"
            self._t0 = now
            self.get_logger().info("[TD] Recovery stand")
            return

        # ── STAND: wait for stand-up ──
        if self._phase == "STAND":
            if now - self._t0 >= self.STAND_DUR:
                yaw = self._get_yaw()
                if yaw is not None:
                    self._start_yaw = yaw
                    self._phase = "CALC"
                    self.get_logger().info(
                        f"[TD] Standing done, start_yaw={math.degrees(yaw):.1f}°")
            return

        # ── CALC: compute absolute target yaw ──
        if self._phase == "CALC":
            rel = math.radians(TARGET_ANGLES_DEG[self._target_idx])
            self._target_yaw = normalize_angle(self._start_yaw + rel)
            self.get_logger().info(
                f"[TD] Turn #{self._target_idx+1}: "
                f"{TARGET_ANGLES_DEG[self._target_idx]:.0f}°  "
                f"target_yaw={math.degrees(self._target_yaw):.1f}°")
            self._phase = "TURN"
            return

        # ── TURN: P-control rotation ──
        if self._phase == "TURN":
            yaw = self._get_yaw()
            if yaw is None:
                return
            err = normalize_angle(self._target_yaw - yaw)
            err_deg = math.degrees(err)

            if abs(err) < self.TURN_TOL:
                self._ctrl.send_cmd(
                    MODE_LOCOMOTION, gait_id=GAIT_STAND,
                    vel_des=[0.0, 0.0, 0.0])
                self.get_logger().info(
                    f"[TD] Turn done! err={err_deg:.1f}°")
                self._start_yaw = yaw
                self._target_idx += 1
                if self._target_idx >= len(TARGET_ANGLES_DEG):
                    self._ctrl.stop()
                    self.get_logger().info("[TD] ALL DONE!")
                    self._phase = "DONE"
                else:
                    self._t0 = now
                    self._phase = "CALC"
                return

            # P-control: yaw_rate proportional to error, clamped
            yaw_rate = self.Kp * err
            yaw_rate = max(-self.MAX_YAW_RATE,
                           min(self.MAX_YAW_RATE, yaw_rate))
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[0.0, 0.0, yaw_rate],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self._phase == "DONE":
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TurnDemo()
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
