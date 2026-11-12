@echo off
setlocal EnableExtensions

title Stop All (Hard Stop)
echo ============================================
echo   Stop All + Stop Container
echo ============================================
echo.

echo [1/4] Stop related processes inside container (best effort)...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pkill -9 -f gzserver || true; pkill -9 -f gzclient || true; pkill -9 -f gazebo || true; pkill -9 -f launchsim || true; pkill -9 -f yellow_line_fsm_demo || true; pkill -9 -f motion_ctrl || true; pkill -9 -f forward_turn_seek_orange || true; pkill -9 -f ball_color_detector || true; pkill -9 -f stage3_sprint || true; pkill -9 -f ros2 || true; pkill -9 -f python3 || true'" >nul 2>&1

echo [2/4] Hard stop container...
wsl -u root -e bash -lc "docker stop cyberdog_sim >/dev/null" >nul 2>&1

echo [3/4] Verify container status...
wsl -u root -e bash -lc "docker ps -a --filter name=cyberdog_sim --format 'cyberdog_sim: {{.Status}}'"

echo [4/4] Done.
echo.
pause
