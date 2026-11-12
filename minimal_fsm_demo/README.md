# Minimal FSM Demo (Sim + Locomotion)

This package demonstrates a thin top-level FSM running on top of
official cyberdog sim + locomotion.

## FSM states

- `INIT`: call `prepare()` and switch locomotion mode
- `GO_STRAIGHT`: walk forward at low speed
- `TURN_LEFT`: turn in place for a short duration
- `PAUSE`: stop briefly, then loop

## Build

```bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
colcon build --merge-install --packages-select minimal_fsm_demo
source install/setup.bash
```

## Run

Start official simulation and locomotion first, then start this demo:

```bash
ros2 launch minimal_fsm_demo yellow_line_fsm_demo.launch.py
```

By default the node uses `lcm_gamepad` backend (official sim control path).

Optional: fallback to `/cmd_vel` backend:

```bash
ros2 run minimal_fsm_demo yellow_line_fsm_demo --ros-args -p locomotion_backend:=cmd_vel
```
