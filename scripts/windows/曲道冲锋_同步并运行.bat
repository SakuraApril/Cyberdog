@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Stage 3 - Waypoint Navigation
echo ============================================
echo   Stage 3 - Waypoint Navigation
echo ============================================
echo.

echo [1/2] Kill old stage3 processes...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pkill -9 -f stage3_sprint || true'"

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [2/2] Running stage3 waypoint nav...
echo.
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 -m minimal_fsm_demo.stage3_sprint'"
pause
