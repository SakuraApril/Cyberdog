#!/usr/bin/env python3
"""
PID closed-loop rotation: turn 180 degrees.

Uses LCM global_to_robot (port 7667) for yaw feedback at 50Hz.
PID controller with anti-windup and low-pass filter.
Reference: task_xunxian.py execute_rotation()
"""

import math
import time
import threading

import lcm

from minimal_fsm_demo.lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from minimal_fsm_demo.lcm_types.localization_lcmt import localization_lcmt

# ── LCM channels ──
CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
RPY_URL = "udpm://239.255.76.67:7667?ttl=255"
CMD_CHANNEL = "robot_control_cmd"
RPY_CHANNEL = "global_to_robot"

# ── Robot constants ──
MODE_LOCOMOTION = 11
MODE_RECOVERY_STAND = 12
MODE_PURE_DAMPER = 7
GAIT_STAND = 1
GAIT_TROT_SLOW = 27
CONTACT_ALL = 15

# ── Control parameters ──
TARGET_ANGLE_DEG = 180.0
STEP_HEIGHT = 0.06
STAND_DUR = 5.0
CONTROL_DT = 0.02           # 50Hz
YAW_RATE_LIMIT = 0.8        # rad/s clamp
STABILITY_SAMPLES = 50      # 1 second at 50Hz
STABILITY_TOL = math.radians(0.5)  # 0.5 degree


def normalize_angle(a):
    return a - (2 * math.pi) * math.floor((a + math.pi) / (2 * math.pi))


class PIDControllerRad:
    """PID with anti-windup and low-pass filtered derivative."""

    def __init__(self, Kp=0.8, Ki=0.02, Kd=0.3,
                 output_limit=1.0, integral_limit=0.3, filter_alpha=0.2):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.filter_alpha = filter_alpha

        self._integral = 0.0
        self._prev_measurement = None
        self._prev_error = 0.0
        self._filtered_d = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None
        self._prev_error = 0.0
        self._filtered_d = 0.0

    def calculate(self, target, measurement):
        error = normalize_angle(target - measurement)

        # Proportional
        p = self.Kp * error

        # Integral with anti-windup: only accumulate when close
        if abs(error) < math.radians(5):
            self._integral += error * CONTROL_DT
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral))
        i = self.Ki * self._integral

        # Derivative (filtered, on measurement to avoid kick)
        if self._prev_measurement is not None:
            raw_d = (measurement - self._prev_measurement) / CONTROL_DT
            self._filtered_d = (self.filter_alpha * raw_d +
                                (1 - self.filter_alpha) * self._filtered_d)
        d = -self.Kd * self._filtered_d

        self._prev_measurement = measurement
        self._prev_error = error

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))


class TurnPidDemo:

    def __init__(self):
        # LCM instances
        self.lcm_cmd = lcm.LCM(CMD_URL)
        self.lcm_rpy = lcm.LCM(RPY_URL)

        # State
        self.yaw = 0.0
        self._life_count = 0
        self._cmd_msg = robot_control_cmd_lcmt()
        self._running = True

        # Subscribe to RPY feedback
        self.lcm_rpy.subscribe(RPY_CHANNEL, self._on_rpy)
        threading.Thread(target=self._rpy_loop, daemon=True).start()

        # Send heartbeat at 200Hz (every 5ms like reference)
        threading.Thread(target=self._send_loop, daemon=True).start()

        # PID
        self.pid = PIDControllerRad(
            Kp=0.8, Ki=0.02, Kd=0.3,
            output_limit=1.0, integral_limit=0.3, filter_alpha=0.2)

        print(f"[TurnPID] Target: {TARGET_ANGLE_DEG}°")
        print(f"[TurnPID] PID: Kp=0.8 Ki=0.02 Kd=0.3")
        print(f"[TurnPID] Yaw rate limit: ±{YAW_RATE_LIMIT} rad/s")
        print(f"[TurnPID] Tolerance: {math.degrees(STABILITY_TOL):.1f}°")

    def _on_rpy(self, channel, data):
        msg = localization_lcmt().decode(data)
        self.yaw = msg.rpy[2]

    def _rpy_loop(self):
        while self._running:
            self.lcm_rpy.handle()
            time.sleep(0.02)

    def _send_loop(self):
        while self._running:
            self.lcm_cmd.publish(CMD_CHANNEL, self._cmd_msg.encode())
            time.sleep(0.005)

    def send_cmd(self, mode, gait_id=0, vel_des=None, duration=0,
                 contact=CONTACT_ALL, step_height=None):
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

    def run(self):
        # 1. Recovery stand
        print("[TurnPID] Recovery stand...")
        self.send_cmd(MODE_RECOVERY_STAND, contact=0, duration=5000,
                      step_height=[STEP_HEIGHT, STEP_HEIGHT])
        time.sleep(STAND_DUR)

        # 2. Record start yaw and compute target
        start_yaw = normalize_angle(self.yaw)
        target_yaw = normalize_angle(start_yaw + math.radians(TARGET_ANGLE_DEG))
        print(f"[TurnPID] start_yaw={math.degrees(start_yaw):.1f}°  "
              f"target={math.degrees(target_yaw):.1f}°")

        # 3. PID rotation loop
        self.pid.reset()
        error_history = []
        start_time = time.time()

        while self._running and (time.time() - start_time < 15.0):
            current = normalize_angle(self.yaw)
            error = normalize_angle(target_yaw - current)
            error_deg = math.degrees(abs(error))

            error_history.append(abs(error))
            if len(error_history) > 200:
                error_history.pop(0)

            # Stability check
            if len(error_history) >= STABILITY_SAMPLES:
                recent = error_history[-STABILITY_SAMPLES:]
                avg_err = sum(recent) / len(recent)
                if avg_err < STABILITY_TOL * 1.5 and error_deg < math.degrees(STABILITY_TOL) * 2:
                    print(f"[TurnPID] Stable! avg={math.degrees(avg_err):.2f}°  "
                          f"cur={error_deg:.2f}°  "
                          f"yaw={math.degrees(current):.1f}°")
                    break

            yaw_rate = self.pid.calculate(target_yaw, current)
            yaw_rate = max(-YAW_RATE_LIMIT, min(YAW_RATE_LIMIT, yaw_rate))

            self.send_cmd(MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
                          vel_des=[0.0, 0.0, yaw_rate],
                          step_height=[STEP_HEIGHT, STEP_HEIGHT])

            time.sleep(CONTROL_DT)

        # 4. Stop
        self.send_cmd(MODE_LOCOMOTION, gait_id=GAIT_STAND,
                      vel_des=[0.0, 0.0, 0.0])
        time.sleep(0.5)

        final_yaw = normalize_angle(self.yaw)
        total_turned = normalize_angle(final_yaw - start_yaw)
        print(f"[TurnPID] Done! turned={math.degrees(total_turned):.1f}°  "
              f"final_yaw={math.degrees(final_yaw):.1f}°  "
              f"time={time.time()-start_time:.1f}s")

        self.stop()


def main():
    demo = TurnPidDemo()
    try:
        demo.run()
    except KeyboardInterrupt:
        demo.stop()


if __name__ == "__main__":
    main()
