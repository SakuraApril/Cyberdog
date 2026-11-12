#!/usr/bin/env python3
"""
Path: forward 120cm -> left 90° -> forward 260cm.
All rotations use PID closed-loop via LCM RPY feedback.
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
GAIT_STAND = 1
GAIT_TROT = 26
GAIT_TROT_SLOW = 27
CONTACT_ALL = 15

CONTROL_DT = 0.02
YAW_RATE_LIMIT = 0.8
STABILITY_TOL = math.radians(0.5)


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


class State(enum.Enum):
    INIT = "INIT"
    STAND_UP = "STAND_UP"
    WALK_1 = "WALK_1"
    TURN_1 = "TURN_1"
    WALK_2 = "WALK_2"
    TURN_2 = "TURN_2"
    WALK_3 = "WALK_3"
    TURN_3 = "TURN_3"
    WALK_4 = "WALK_4"
    TURN_4 = "TURN_4"
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

    def _rpy_loop(self):
        while self._running:
            self._lcm_rpy.handle()
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
            if self._lcm_cmd and msg:
                self._lcm_cmd.publish(CMD_CHANNEL, msg.encode())
            time.sleep(0.1)


class MotionCtrl(Node):

    WALK_SPEED = 0.3
    STEP_HEIGHT = 0.06
    STAND_DUR = 5.0
    DIST_1 = 1.20             # 120cm
    DIST_2 = 2.70             # 270cm
    DIST_3 = 3.95             # 395cm
    DIST_4 = 5.00             # 300cm

    def __init__(self):
        super().__init__("motion_ctrl")
        self.state = State.INIT
        self._t0 = 0.0
        self._ctrl = RobotCtrl(self)
        self._pid = PIDControllerRad(
            Kp=0.8, Ki=0.02, Kd=0.3,
            output_limit=1.0, integral_limit=0.3, filter_alpha=0.2)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[M] Path: 120cm -> L90° -> 270cm -> R120° -> 395cm -> L135° -> 300cm -> R70°")

    def _pid_rotate(self, target_angle_deg, label):
        start_yaw = normalize_angle(self._ctrl.yaw)
        target_yaw = normalize_angle(start_yaw + math.radians(target_angle_deg))
        self.get_logger().info(
            f"[PID] {label}: start={math.degrees(start_yaw):.1f}° "
            f"target={math.degrees(target_yaw):.1f}°")
        self._pid.reset()
        error_history = []
        t0 = time.time()
        while self._ctrl._running and (time.time() - t0 < 15.0):
            current = normalize_angle(self._ctrl.yaw)
            error = normalize_angle(target_yaw - current)
            error_history.append(abs(error))
            if len(error_history) > 200:
                error_history.pop(0)
            if len(error_history) >= 50:
                recent = error_history[-50:]
                avg_err = sum(recent) / len(recent)
                if avg_err < STABILITY_TOL * 1.5 and abs(error) < STABILITY_TOL * 2:
                    self.get_logger().info(
                        f"[PID] {label} done: err={math.degrees(error):.2f}° "
                        f"time={time.time()-t0:.1f}s")
                    break
            yaw_rate = self._pid.calculate(target_yaw, current)
            yaw_rate = max(-YAW_RATE_LIMIT, min(YAW_RATE_LIMIT, yaw_rate))
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                vel_des=[0.0, 0.0, yaw_rate],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            time.sleep(CONTROL_DT)
        self._ctrl.send_cmd(
            MODE_LOCOMOTION, gait_id=GAIT_STAND,
            vel_des=[0.0, 0.0, 0.0])
        time.sleep(0.5)

    def control_loop(self):
        now = time.time()

        if self.state == State.INIT:
            self._ctrl.send_cmd(
                MODE_RECOVERY_STAND, contact=0, duration=5000,
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            self.state = State.STAND_UP
            self._t0 = now
            self.get_logger().info("[M] Recovery stand")
            return

        if self.state == State.STAND_UP:
            if now - self._t0 >= self.STAND_DUR:
                self.state = State.WALK_1
                self._t0 = now
                self.get_logger().info(f"[M] Walk {self.DIST_1*100:.0f}cm")
            return

        if self.state == State.WALK_1:
            if now - self._t0 >= self.DIST_1 / self.WALK_SPEED:
                self.state = State.TURN_1
                self.get_logger().info("[M] -> TURN_1: left 90°")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.TURN_1:
            self._pid_rotate(90.0, "Turn left 90°")
            self.state = State.WALK_2
            self._t0 = time.time()
            self.get_logger().info(f"[M] Walk {self.DIST_2*100:.0f}cm")
            return

        if self.state == State.WALK_2:
            if now - self._t0 >= self.DIST_2 / self.WALK_SPEED:
                self.state = State.TURN_2
                self.get_logger().info("[M] -> TURN_2: right 120°")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.TURN_2:
            self._pid_rotate(-143.0, "Turn right 120°")
            self.state = State.WALK_3
            self._t0 = time.time()
            self.get_logger().info(f"[M] Walk {self.DIST_3*100:.0f}cm")
            return

        if self.state == State.WALK_3:
            if now - self._t0 >= self.DIST_3 / self.WALK_SPEED:
                self.state = State.TURN_3
                self.get_logger().info("[M] -> TURN_3: left 135°")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.TURN_3:
            self._pid_rotate(135.0, "Turn left 135°")
            self.state = State.WALK_4
            self._t0 = time.time()
            self.get_logger().info(f"[M] Walk {self.DIST_4*100:.0f}cm")
            return

        if self.state == State.WALK_4:
            if now - self._t0 >= self.DIST_4 / self.WALK_SPEED:
                self.state = State.TURN_4
                self.get_logger().info("[M] -> TURN_4: right 70°")
                return
            self._ctrl.send_cmd(
                MODE_LOCOMOTION, gait_id=GAIT_TROT,
                vel_des=[self.WALK_SPEED, 0.0, 0.0],
                step_height=[self.STEP_HEIGHT, self.STEP_HEIGHT])
            return

        if self.state == State.TURN_4:
            self._pid_rotate(-70.0, "Turn right 70°")
            self._ctrl.stop()
            self.get_logger().info("[M] DONE!")
            self.state = State.DONE
            return

        if self.state == State.DONE:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MotionCtrl()
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
