@echo off
setlocal EnableExtensions
title Cyberdog Gazebo (Interactive)
echo ============================================
echo   Cyberdog Gazebo (Interactive Mode) +LIDAR
echo ============================================
echo [1/4] Starting container...
wsl -u root -e bash -lc "docker start cyberdog_sim"
if errorlevel 1 (
  echo [ERROR] Failed to start container cyberdog_sim.
  pause
  exit /b 1
)
echo [2/4] Cleaning stale gazebo processes...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'pkill -9 -f gzserver >/dev/null 2>&1 || true; pkill -9 -f gzclient >/dev/null 2>&1 || true; pkill -9 -f gazebo >/dev/null 2>&1 || true'"
echo [3/4] Entering container and launching simulator...
echo [INFO] This window will stay attached like manual Ubuntu commands.
echo [4/4] Running launchsim.py ...
wsl -u root -e bash -lc "docker exec -it -e DISPLAY=:0 -e QT_QPA_PLATFORM=xcb -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e LD_LIBRARY_PATH=/usr/lib/wsl/lib:/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/lib/x86_64-linux-gnu:/usr/lib:/lib cyberdog_sim /bin/bash -lc 'cd /home/cyberdog_sim && python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py'"
echo.
echo [INFO] launchsim.py exited. Press any key to close.
pause
