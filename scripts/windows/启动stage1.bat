@echo off
title Stage1 Control
echo [1/2] Starting container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"
if errorlevel 1 (
  echo [ERROR] Container start failed.
  pause
  exit /b 1
)
echo [2/2] Running stage1 node...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.yellow_line_fsm_demo'"
pause
