@echo off
echo ============================================
echo   ?? motion_manager
echo ============================================
echo.

echo [1/2] ???????...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"
if errorlevel 1 (
    echo [ERROR] Container start failed!
    pause
    exit /b 1
)

echo [2/2] ?? motion_manager...
echo.
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_ws && source /opt/ros/galactic/setup.bash && source /home/cyberdog_ws/install/setup.bash && ros2 run motion_manager motion_manager'"
pause
