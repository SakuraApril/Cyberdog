@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Stone Walk - Sync and Run
echo ============================================
echo   Stone Path 400cm - Walk Gait Direct
echo ============================================
echo.

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [RUN] Running stone walk...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 -m minimal_fsm_demo.yellow_line_fsm_demo'"
pause
