@echo off
title Build minimal_fsm_demo
echo ============================================
echo   Build minimal_fsm_demo package only
echo ============================================
echo.

echo [1/2] Starting container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"
if errorlevel 1 (
  echo [ERROR] Container start failed.
  pause
  exit /b 1
)

echo [2/2] Building...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && colcon build --merge-install --symlink-install --packages-select minimal_fsm_demo'"
echo.
echo Done.
pause
