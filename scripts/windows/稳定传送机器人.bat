@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Stable Teleport Robot

REM Usage:
REM   ??????????
REM   ??????: stable_teleport.bat x y yaw [z]

set "TX=%~1"
set "TY=%~2"
set "TYAW=%~3"
set "TZ=%~4"

if "%TX%"=="" set "TX=3.0446"
if "%TY%"=="" set "TY=-0.173699"
if "%TYAW%"=="" set "TYAW=1.48672"
if "%TZ%"=="" set "TZ=0.20"

echo ============================================
echo   Stable Teleport
echo ============================================
echo Target: x=%TX% y=%TY% z=%TZ% yaw=%TYAW%
echo.

echo [1/5] Starting container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"
if errorlevel 1 (
  echo [ERROR] Container start failed.
  pause
  exit /b 1
)

echo [2/5] Pause physics...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && (ros2 service call /gazebo/pause_physics std_srvs/srv/Empty {} || true)'" >nul 2>&1

echo [3/5] Teleport (attempt 1)...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'gz model -m robot -x %TX% -y %TY% -z %TZ% -Y %TYAW% 2>&1'"
timeout /t 1 >nul

echo [4/5] Unpause physics...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && (ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty {} || true)'" >nul 2>&1
timeout /t 1 >nul

echo [5/5] Check pose...
set "POSE="
for /f "delims=" %%P in ('wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'gz model -m robot -p 2>&1'"') do set "POSE=%%P"
echo Pose: %POSE%

REM Simple roll/pitch sanity check, retry once if unstable
for /f "tokens=1,2,3,4,5,6" %%a in ("%POSE%") do (
  set "R=%%d"
  set "P=%%e"
)

wsl -u root -e bash -lc "python3 - <<'PY'
import math
r=float(r'''%R%''') if r'''%R%''' else 0.0
p=float(r'''%P%''') if r'''%P%''' else 0.0
print('UNSTABLE' if abs(r)>0.35 or abs(p)>0.35 else 'OK')
PY" > "%TEMP%\teleport_check.txt" 2>nul
findstr /c:"UNSTABLE" "%TEMP%\teleport_check.txt" >nul
if not errorlevel 1 (
  echo [WARN] Pose unstable, retrying once...
  wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && (ros2 service call /gazebo/pause_physics std_srvs/srv/Empty {} || true) && gz model -m robot -x %TX% -y %TY% -z %TZ% -Y %TYAW% 2>&1 && (ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty {} || true) && gz model -m robot -p 2>&1'"
)

echo.
echo Done.
pause
