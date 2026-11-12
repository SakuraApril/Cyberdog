@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Stage2 v2 - Grid Hunt + Vision
echo ============================================
echo   Stage2 v2 - Grid Hunt + Orange Vision
echo ============================================
echo.

echo [1/4] Kill old stage2/vision processes...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pkill -9 -f stage2_grid_hunt || true; pkill -9 -f ball_color_detector || true; pkill -9 -f stage2_orange_scan || true'"

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [2/4] Start vision node (background)...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && nohup python3 -m minimal_fsm_demo.ball_color_detector --ros-args -p image_topic:=/rgb_camera/image_raw >/tmp/ball_color_detector.log 2>&1 &'"

echo [3/4] Start stage2_grid_hunt (background)...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && nohup python3 -m minimal_fsm_demo.stage2_grid_hunt >/tmp/stage2_grid_hunt.log 2>&1 &'"

echo [4/4] Verify processes...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pgrep -af ball_color_detector || echo [WARN] vision not running; pgrep -af stage2_grid_hunt || echo [WARN] stage2 not running'"

echo.
echo Done. Check logs:
echo   vision: docker exec cyberdog_sim tail -f /tmp/ball_color_detector.log
echo   stage2: docker exec cyberdog_sim tail -f /tmp/stage2_grid_hunt.log
pause
