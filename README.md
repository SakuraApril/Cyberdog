# CyberDog FSM Navigation

Autonomous multi-stage navigation stack for the Xiaomi CyberDog quadruped —
FSM-based stage runner, PID closed-loop heading control over LCM, HSV + contour
vision, per-stage failure fallbacks.

> 2026 全国大学生计算机系统能力大赛 · 小米杯智能系统创新设计赛　**全国三等奖**

## 这个项目在做什么

一整条闭环：位姿反馈（LCM）进来 → 状态机决策 → 速度指令（LCM）下发 → 运控执行 → 再回到位姿判断“到没到点”。

每个赛段是一个独立的状态机文件（`stage3_sprint.py`、`stage5.py` …），共享同一套基础设施：

- `RobotCtrl` —— LCM 指令发送 + 位姿订阅 + 心跳保活（三条线程，锁保护共享指令帧）
- `PIDControllerRad` —— 航向角闭环，带积分分离、微分滤波、输出限幅
- 统一的兜底约定 —— 到位距离 / 航向误差 / 超时三重判定，任一异常都有降级路径

## 架构

| 层 | 实现 |
|---|---|
| 决策 | Python 状态机，20 Hz 决策环，每赛段一个状态集合 |
| 控制 | 50 Hz PID 航向闭环，输出限幅 + 临近目标降速抑制超调 |
| 通信 | LCM over UDP 组播（`udpm://239.255.76.67:7671` 下发 / `:7667` 反馈），10 Hz 心跳 |
| 感知 | ROS 2 订阅 RGB，OpenCV HSV 阈值 + 形态学开闭 + 轮廓圆度评分 |
| 硬件 | Jetson NX（决策 + 视觉）+ MR813（实时运控）双板分工 |

LCM 通道与模式常量都集中在各文件头部，方便对着官方 `robot_control_cmd` 协议改参数：

```
MODE_PURE_DAMPER = 7     # 阻尼（软停）
MODE_LOCOMOTION  = 11    # 行走
MODE_RECOVERY_STAND = 12 # 恢复站立
GAIT_TROT = 26 / GAIT_TROT_SLOW = 27
```

## 关键设计

**1. 航向闭环为什么不用固定时长转向**
早期是「转 1.55 s ≈ 90°」的开环定时，地面摩擦一变就偏。改成 PID 闭环后，判稳条件是「误差 < 0.5° **且** 角速度 < 0.08 rad/s」——只看误差会在还在转的时候误判完成。

**2. 到位判定用位姿而不是时间**
主判据是 `hypot(dx, dy) >= 目标距离`；时间只作为兜底（`WALK_TIMEOUT_FACTOR = 1.8`），避免定位失效时卡死。

**3. 视觉只做「筛」，不做「认」**
HSV 阈值给候选，再用面积、圆度、ROI 位置三重过滤（裁掉画面上 25% 的远景区域，减少道具误检）。误检的直接代价是机器人跑偏，所以宁可漏检也不要误检 —— 漏检会退化成搜索行为。

**4. 每段都有失败路径**
定位丢失 → 盲走；目标丢失 → 原地搜索；姿态异常 → 恢复站立；超时 → 降级到下一段。目的是「可重复复现」，而不是「能跑通一次」。

## 仓库结构

```
minimal_fsm_demo/          ROS 2 包（ament_python）
  minimal_fsm_demo/
    yellow_line_fsm_demo.py  赛段 1：转 180° → 后退 → 转 90°
    motion_ctrl.py           基础行走 / PID 转向的最小实现
    stage2_grid_hunt.py      赛段 2：多航点
    stage3_sprint.py         赛段 3：曲道
    stage4.py                赛段 4：蹲姿航点
    stage5.py                赛段 5：长距离直行 + 逐段姿态修正
    stage6.py / stage6_o.py  赛段 6：多航点 + 起跳
    stage7.py                跳跃 1 m
    stage8.py                视觉追球 + 推到终点 + 趴下
    lcm_types/               LCM 消息结构（robot_control_cmd / localization …）
    *_demo.py, *_monitor.py  转向、站立、相机、TF 等单项调试工具
  launch/                    launch 文件（可选视觉节点）
scripts/windows/             26 个一键脚本（同步代码到 Docker / 启动仿真 / 单段运行）
docs/数据流.html             双通道控制数据流图（Mermaid）
```

## 编译与运行

```bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
colcon build --merge-install --packages-select minimal_fsm_demo
source install/setup.bash

# 默认走官方 lcm_gamepad 控制链路
ros2 launch minimal_fsm_demo yellow_line_fsm_demo.launch.py

# 需要退回到 /cmd_vel 后端时
ros2 run minimal_fsm_demo yellow_line_fsm_demo --ros-args -p locomotion_backend:=cmd_vel
```

单个赛段直接按模块跑：

```bash
python3 -m minimal_fsm_demo.stage5
```

## 关于 Windows 一键脚本

`scripts/windows/*.bat` 是比赛期间的工作流：把本地改动 `docker cp` 进仿真容器、清 `pyc`、source 环境、按模块启动。
这些脚本假设「WSL + Docker + `cyberdog_sim` 容器」的本地环境，换机器需要改 `%SRC%` 与容器名。放在这里是作为调试流程的记录。

## 已知限制

- 位姿依赖仿真器 / 运控上报的 `global_to_robot`，真机长时间运行会有累积漂移，靠每段重置基准压住
- 视觉参数（HSV 阈值、ROI 比例）是针对比赛场地调的，换光照要重标
- `stage6_o.py`、`stage2_orange_scan.py` 等是并行试的备用路线，未进最终比赛流程

---

作者：鲁品奕 · 武汉大学 软件工程
