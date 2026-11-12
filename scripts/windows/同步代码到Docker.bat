@echo off
title Sync & Build
echo [1/2] Starting container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"
if errorlevel 1 (
  echo [ERROR] Container start failed.
  pause
  exit /b 1
)
echo [2/2] Building package...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && colcon build --merge-install --packages-select minimal_fsm_demo'"
pause
