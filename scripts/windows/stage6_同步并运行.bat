@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Stage6 - Vision Ball Chase + Kick
echo ============================================
echo   Stage6 - Vision Ball Chase + Kick
echo ============================================
echo.

echo [1/3] Kill old stage6 processes...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pkill -9 -f stage6 || true'"

echo [2/3] Syncing files...
call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)
wsl -u root -e bash -lc "docker cp /mnt/d/minimal_fsm_demo/minimal_fsm_demo/stage6.py cyberdog_sim:/home/cyberdog_sim/install/lib/python3.8/site-packages/minimal_fsm_demo/stage6.py"

echo [3/3] Running stage6 vision chase + kick...
echo.
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 -m minimal_fsm_demo.stage6'"
pause
