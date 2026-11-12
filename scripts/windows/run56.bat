@echo off
title Run Stage5 + Stage6
echo ============================================
echo   Run Stage5 + Stage6
echo ============================================
echo.

set SRC=/mnt/d/minimal_fsm_demo/minimal_fsm_demo
set DST=/home/cyberdog_sim/install/lib/python3.8/site-packages/minimal_fsm_demo
set PYC=%DST%/__pycache__

echo [Sync] Copying files to container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null 2>&1"
wsl -u root -e bash -lc "docker cp %SRC%/stage5.py cyberdog_sim:%DST%/stage5.py"
wsl -u root -e bash -lc "docker cp %SRC%/stage6.py cyberdog_sim:%DST%/stage6.py"
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'rm -f %PYC%/stage5.*.pyc %PYC%/stage6.*.pyc'"
echo [Sync] Done.
echo.

echo ============================================
echo   Stage 5/6: stage5
echo ============================================
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.stage5'"
if errorlevel 1 echo [WARN] Stage5 exit code: %errorlevel%

echo.
echo ============================================
echo   Stage 6/6: stage6
echo ============================================
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.stage6'"
if errorlevel 1 echo [WARN] Stage6 exit code: %errorlevel%

echo.
echo ============================================
echo   Stage5 + Stage6 completed!
echo ============================================
pause
