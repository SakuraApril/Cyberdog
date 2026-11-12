#!/usr/bin/env python3
"""
Turn 180°, walk backward 3m, turn right 90°.
PID closed-loop rotation using LCM RPY feedback.

FSM: INIT -> STAND_UP -> TURN_180 -> STABILIZE -> BACKWARD_WALK -> TURN_R90 -> STABILIZE_B -> STOP
"""

import enum
import math
import time
import threading

import rclpy
from rclpy.node import Node

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
GAIT_DEFAULT = 0
GAIT_STAND = 1
GAIT_TROT = 26
GAIT_TROT_SLOW = 27
CONTACT_ALL = 15

CONTROL_DT = 0.02            # 50Hz PID loop
YAW_RATE_LIMIT = 0.8         # rad/s
STABILITY_TOL = math.radians(0.5)  # 0.5 degree
STOP_RATE_TOL = 0.08         # rad/s, require low turn rate before finish


def normalize_angle(a):
    return a - (2 * math.pi) * math.floor((a + math.pi) / (2 * math.pi))


class PIDControllerRad:
    def __init__(self, Kp=0.8, Ki=0.02, Kd=0.3,
                 output_limit=1.0, integral_limit=0.3, filter_alpha=0.2):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.filter_alpha = filter_alpha
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._filtered_d = 0.0

    def calculate(self, target, measurement):
        error = normalize_angle(target - measurement)
        p = self.Kp * error
        if abs(error) < math.radians(5):
            self._integral += error * CONTROL_DT
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral))
        i = self.Ki * self._integral
        if self._prev_meas is not None:
            raw_d = (measurement - self._prev_meas) / CONTROL_DT
            self._filtered_d = (self.filter_alpha * raw_d +
                                (1 - self.filter_alpha) * self._filtered_d)
        d = -self.Kd * self._filtered_d
        self._prev_meas = measurement
        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))


class DemoState(enum.Enum):
    INIT = "INIT"
    STAND_UP = "STAND_UP"
    TURN_180 = "TURN_180"
    STABILIZE_A = "STABILIZE_A"
    BACKWARD_WALK = "BACKWARD_WALK"
    TURN_R90 = "TURN_R90"
    STABILIZE_B = "STABILIZE_B"
    STOP = "STOP"


class RobotCtrl:
    """LCM command sender + RPY receiver."""

    def __init__(self, node: Node):
        self._node = node
        self._lcm_cmd = None
        self._lcm_rpy = None
        self._lock = threading.Lock()
        self._cmd_msg = robot_control_cmd_lcmt()
        self._running = True
        self._life_count = 0

        # RPY feedback
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

        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        self._node.get_logger().info(f"[RobotCtrl] LCM ready")

    def _on_rpy(self, channel, data):
        msg = localization_lcmt().decode(data)
        self.yaw = msg.rpy[2]
        # Compatible with different localization_lcmt layouts.
        if hasattr(msg, "p") and len(msg.p) >= 2:
            self.pos_x = float(msg.p[0])
            self.pos_y = float(msg.p[1])
            self.has_pose = True
        elif hasattr(msg, "xyz") and len(msg.xyz) >= 2:
            self.pos_x = float(msg.xyz[0])
            self.pos_y = float(msg.xyz[1])
            self.has_pose = True

    def _rpy_loop(self):
        while self._running:
            try:
                self._lcm_rpy.handle()
            except Exception as exc:
                self._node.get_logger().warn(f"[RobotCtrl] RPY handle error: {exc}")
            time.sleep(0.02)

    def send_cmd(self, mode: int, gait_id: int = GAIT_DEFAULT,
                 vel_des: list = None, rpy_des: list = None,
                 pos_des: list = None, step_height: list = None,
                 duration: int = 0, contact: int = CONTACT_ALL) -> None:
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
            if pos_des is not None:
                for i, v in enumerate(pos_des[:3]):
                    msg.pos_des[i] = v
            if step_height is not None:
                for i, s in enumerate(step_height[:2]):
                    msg.step_height[i] = s
            self._cmd_msg = msg

    def stop(self) -> None:
        self.send_cmd(MODE_PURE_DAMPER, duration=4000)
        time.sleep(0.1)
        self._running = False

    def _heartbeat_loop(self) -> None:
        while self._running:
            with self._lock:
                msg = self._cmd_msg
            if self._lcm_cmd is not None and msg is not None:
                self._lcm_cmd.publish(CMD_CHANNEL, msg.encode())
            time.sleep(0.1)


class YellowLineFsmDemo(Node):

    WALK_SPEED = -0.3
    WALK_DISTANCE = 2.9
    STEP_HEIGHT = 0.06
    STAND_DURATION = 1.5
    STABILIZE_DURATION = 1.0
    WALK_TIMEOUT_FACTOR = 1.8

    def __init__(self):
        super().__init__("yellow_line_fsm_demo")
        self.state = DemoState.INIT
        self._phase_start_time = 0.0
        self._ctrl = RobotCtrl(self)
        self._pid = PIDControllerRad(
            Kp=0.8, Ki=0.02, Kd=0.3,
            output_limit=1.0, integral_limit=0.3, filter_alpha=0.2)
        self._walk_start_pos = None
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[StonePath] PID closed-loop rotation")

    def _pid_rotate(self, target_angle_deg: float, label: str):
        """Blocking PID rotation. Returns when stable."""
        start_yaw = normalize_angle(self._ctrl.yaw)
        target_yaw = normalize_angle(start_yaw + math.radians(target_angle_deg))
        self.get_logger().info(
            f"[PID] {label}: start={math.degrees(start_yaw):.1f}° "
            f"target={math.degrees(target_yaw):.1f}°")

        self._pid.reset()
        t0 = time.time()
        prev_yaw = normalize_angle(self._ctrl.yaw)
        filtered_yaw_rate = 0.0

        while self._ctrl._running and (time.time() - t0 < 15.0):
            current = normalize_angle(self._ctrl.yaw)
            error = normalize_angle(target_yaw - current)
            # Estimate angular speed from feedback to avoid "done while still spinning".
            raw_rate = normalize_angle(current - prev_yaw) / CONTROL_DT
            filtered_yaw_rate = 0.25 * raw_rate + 0.75 * filtered_yaw_rate
            prev_yaw = current

            # Stability check: small error + low angular speed
            if abs(error) < STABILITY_TOL * 2 and abs(filtered_yaw_rate) < STOP_RATE_TOL:
                self.get_logger().info(
                    f"[PID] {label} done: yaw={math.degrees(current):.1f}° "
                    f"err={math.degrees(error):.2f}° "
                    f"rate={math.degrees(filtered_yaw_rate):.2f}°/s "
                    f"time={time.time()-t0:.1f}s")
                break

            yaw_rate = self._pid.calculate(target_yaw, current)
            # Brake near target to reduce overshoot.
            limit = YAW_RATE_LIMIT if abs(error) > math.radians(8.0) else 0.25
            yaw_rate = max(-limit, min(limit, yaw_rate))
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, yaw_rate],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            time.sleep(CONTROL_DT)

        # Stand still
        self._ctrl.send_cmd(
            MODE_LOCOMOTION, gait_id=GAIT_STAND,
            vel_des=[0.0, 0.0, 0.0])
        time.sleep(0.5)

    def control_loop(self) -> None:
        now = time.time()
        walk_dur_nominal = self.WALK_DISTANCE / abs(self.WALK_SPEED)

        if self.state == DemoState.INIT:
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            self.state = DemoState.STAND_UP
            self._phase_start_time = now
            self.get_logger().info("[Demo] Recovery stand")
            return

        if self.state == DemoState.STAND_UP:
            if now - self._phase_start_time >= self.STAND_DURATION:
                self.state = DemoState.TURN_180
                self.get_logger().info("[Demo] -> TURN_180")
            return

        if self.state == DemoState.TURN_180:
            self._pid_rotate(180.0, "Turn 180°")
            self.state = DemoState.STABILIZE_A
            self._phase_start_time = time.time()
            return

        if self.state == DemoState.STABILIZE_A:
            if now - self._phase_start_time >= self.STABILIZE_DURATION:
                self.state = DemoState.BACKWARD_WALK
                self._phase_start_time = now
                if self._ctrl.has_pose:
                    self._walk_start_pos = (self._ctrl.pos_x, self._ctrl.pos_y)
                else:
                    self._walk_start_pos = None
                self.get_logger().info(f"[Demo] Walk backward {self.WALK_DISTANCE}m")
            return

        if self.state == DemoState.BACKWARD_WALK:
            walked_dist = None
            if self._walk_start_pos is not None and self._ctrl.has_pose:
                dx = self._ctrl.pos_x - self._walk_start_pos[0]
                dy = self._ctrl.pos_y - self._walk_start_pos[1]
                walked_dist = math.hypot(dx, dy)

            # Primary condition: reached target distance based on localization.
            reached_target = (walked_dist is not None and walked_dist >= self.WALK_DISTANCE)
            # Fallback: localization unavailable or severe drift, avoid getting stuck forever.
            timeout_hit = (now - self._phase_start_time >= walk_dur_nominal * self.WALK_TIMEOUT_FACTOR)

            if reached_target or timeout_hit:
                self.state = DemoState.TURN_R90
                if reached_target:
                    self.get_logger().info(
                        f"[Demo] Reached distance {walked_dist:.2f}m -> TURN_R90")
                else:
                    self.get_logger().warn(
                        "[Demo] Walk timeout fallback -> TURN_R90")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == DemoState.TURN_R90:
            self._pid_rotate(-90.0, "Turn right 90°")
            self.state = DemoState.STABILIZE_B
            self._phase_start_time = time.time()
            return

        if self.state == DemoState.STABILIZE_B:
            if now - self._phase_start_time >= self.STABILIZE_DURATION:
                self._ctrl.stop()
                self.get_logger().info("[Demo] DONE!")
                self.state = DemoState.STOP
            return

        if self.state == DemoState.STOP:
            self._ctrl._running = False
            self.get_logger().info("[Stage1] Shutdown complete")
            raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YellowLineFsmDemo()
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
