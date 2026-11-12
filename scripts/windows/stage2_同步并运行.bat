@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Stage2 - Grid Hunt
echo ============================================
echo   Stage2 - Grid Hunt Route
echo ============================================
echo.

echo [1/2] Kill old stage2 processes...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pkill -9 -f stage2_grid_hunt || true; pkill -9 -f ball_color_detector || true; pkill -9 -f stage2_orange_scan || true'"

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/install/lib/python3.8/site-packages/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [2/2] Running stage2...
echo.
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 -m minimal_fsm_demo.stage2_grid_hunt'"
pause
