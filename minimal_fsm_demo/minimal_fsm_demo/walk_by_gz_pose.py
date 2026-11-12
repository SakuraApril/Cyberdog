#!/usr/bin/env python3
"""
Walk a precise distance using Gazebo pose feedback:
- Pose source: `gz model -m robot -p`
- Motion command: Cyberdog LCM locomotion command
"""

import argparse
import math
import re
import subprocess
import time

import lcm

from lcm_types.robot_control_cmd_lcmt import robot_control_cmd_lcmt


LCM_CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
CMD_CHANNEL = "robot_control_cmd"

MODE_LOCOMOTION = 11
MODE_PURE_DAMPER = 7
GAIT_TROT = 26
CONTACT_ALL = 15


class Walker:
    def __init__(self):
        self._lcm = lcm.LCM(LCM_CMD_URL)
        self._life_count = 0

    def send_cmd(self, mode, gait_id=0, vx=0.0, vy=0.0, wz=0.0, duration=0):
        self._life_count = (self._life_count + 1) % 128
        msg = robot_control_cmd_lcmt()
        msg.mode = mode
        msg.gait_id = gait_id
        msg.contact = CONTACT_ALL
        msg.life_count = self._life_count
        msg.duration = duration
        msg.vel_des[0] = float(vx)
        msg.vel_des[1] = float(vy)
        msg.vel_des[2] = float(wz)
        msg.step_height[0] = 0.06
        msg.step_height[1] = 0.06
        self._lcm.publish(CMD_CHANNEL, msg.encode())

    def stop(self):
        for _ in range(5):
            self.send_cmd(MODE_PURE_DAMPER, duration=2000)
            time.sleep(0.05)


def read_robot_xy(timeout_s=5.0, retries=3):
    # Example user command:
    # gz model -m robot -p
    last_err = None
    out = ""
    for _ in range(max(1, retries)):
        try:
            out = subprocess.check_output(
                ["bash", "-lc", "gz model -m robot -p 2>&1"],
                text=True,
                timeout=timeout_s,
            ).strip()
            break
        except Exception as e:
            last_err = e
            time.sleep(0.1)
    else:
        raise RuntimeError(f"gz pose query failed: {last_err}")
    nums = re.findall(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", out)
    if len(nums) < 2:
        raise RuntimeError(f"Cannot parse robot pose from: {out}")
    x = float(nums[0])
    y = float(nums[1])
    return x, y, out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, required=True, help="Target meters, e.g. 3.15")
    parser.add_argument("--speed", type=float, default=0.20, help="Forward speed m/s (positive value)")
    parser.add_argument("--tol", type=float, default=0.03, help="Distance tolerance in meters")
    parser.add_argument("--backward", action="store_true", help="Walk backward")
    args = parser.parse_args()

    if args.distance <= 0:
        raise ValueError("--distance must be > 0")
    if args.speed <= 0:
        raise ValueError("--speed must be > 0")

    walker = Walker()
    direction = -1.0 if args.backward else 1.0

    x0, y0, raw0 = read_robot_xy()
    print(f"[walk_by_gz_pose] start pose: x={x0:.3f}, y={y0:.3f}")
    print(f"[walk_by_gz_pose] raw: {raw0}")
    print(f"[walk_by_gz_pose] target distance={args.distance:.3f}m speed={args.speed:.3f}m/s")

    start_t = time.time()
    last_print_t = 0.0
    try:
        while True:
            walker.send_cmd(MODE_LOCOMOTION, gait_id=GAIT_TROT, vx=direction * args.speed)
            x, y, _ = read_robot_xy()
            d = math.hypot(x - x0, y - y0)
            now = time.time()
            if now - last_print_t > 0.5:
                print(f"[walk_by_gz_pose] dist={d:.3f}/{args.distance:.3f} x={x:.3f} y={y:.3f}")
                last_print_t = now
            if d >= args.distance - args.tol:
                print(f"[walk_by_gz_pose] reached: {d:.3f}m")
                break
            if now - start_t > 120:
                print("[walk_by_gz_pose] timeout, stopping")
                break
            time.sleep(0.05)
    finally:
        walker.stop()
        x1, y1, raw1 = read_robot_xy()
        d1 = math.hypot(x1 - x0, y1 - y0)
        print(f"[walk_by_gz_pose] final pose: x={x1:.3f}, y={y1:.3f}, moved={d1:.3f}m")
        print(f"[walk_by_gz_pose] raw: {raw1}")


if __name__ == "__main__":
    main()
