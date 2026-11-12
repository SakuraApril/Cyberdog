@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Walk Straight 5m + Voice
echo ============================================
echo   Walk Straight 5m + Object Detection
echo ============================================
echo.

call "%~dp0common_sync.bat" "/mnt/d/minimal_fsm_demo/minimal_fsm_demo" "/home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo"
if errorlevel 1 (
  pause
  exit /b 1
)

echo [RUN] Running walk straight...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 -m minimal_fsm_demo.walk_straight'"

echo [POST] Checking detection result...
set "DETECT_TEXT="
for /f "delims=" %%T in ('wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cat /tmp/cyberdog_say.txt 2>/dev/null || true'"') do set "DETECT_TEXT=%%T"
if defined DETECT_TEXT (
    echo Detected: !DETECT_TEXT!
    cscript //nologo "C:\Users\33272\Desktop\say.vbs" "!DETECT_TEXT!"
    wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'rm -f /tmp/cyberdog_say.txt'" >nul 2>&1
) else (
    echo No signal file found.
)
pause
