@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Turn Demo - Closed Loop
echo ============================================
echo   Turn Demo - TF Closed Loop Rotation
echo ============================================
echo.

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [RUN] Running turn demo...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 -m minimal_fsm_demo.turn_demo'"
pause
