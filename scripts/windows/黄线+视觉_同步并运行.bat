@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Yellow Line + Vision

echo ============================================
echo   Yellow Line + Vision (Sync and Run)
echo ============================================
echo.

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [RUN] Starting control + vision launch...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch minimal_fsm_demo yellow_line_with_vision.launch.py'"
pause
