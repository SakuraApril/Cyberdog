@echo off
title Run All 6 Stages
echo ============================================
echo   Run All 6 Stages
echo ============================================
echo.

set SRC=/mnt/d/minimal_fsm_demo/minimal_fsm_demo
set DST=/home/cyberdog_sim/install/lib/python3.8/site-packages/minimal_fsm_demo
set PYC=%DST%/__pycache__

echo [Sync] Copying files to container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null 2>&1"
wsl -u root -e bash -lc "docker cp %SRC%/yellow_line_fsm_demo.py cyberdog_sim:%DST%/yellow_line_fsm_demo.py"
wsl -u root -e bash -lc "docker cp %SRC%/stage2_grid_hunt.py cyberdog_sim:%DST%/stage2_grid_hunt.py"
wsl -u root -e bash -lc "docker cp %SRC%/stage3_sprint.py cyberdog_sim:%DST%/stage3_sprint.py"
wsl -u root -e bash -lc "docker cp %SRC%/stage4.py cyberdog_sim:%DST%/stage4.py"
wsl -u root -e bash -lc "docker cp %SRC%/stage5.py cyberdog_sim:%DST%/stage5.py"
wsl -u root -e bash -lc "docker cp %SRC%/stage6.py cyberdog_sim:%DST%/stage6.py"
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'rm -f %PYC%/*.pyc'"
echo [Sync] Done.
echo.

echo ============================================
echo   Stage 1/6: yellow_line_fsm_demo
echo ============================================
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.yellow_line_fsm_demo'"
if errorlevel 1 echo [WARN] Stage1 exit code: %errorlevel%

echo.
echo ============================================
echo   Stage 2/6: stage2_grid_hunt
echo ============================================
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.stage2_grid_hunt'"
if errorlevel 1 echo [WARN] Stage2 exit code: %errorlevel%

echo.
echo ============================================
echo   Stage 3/6: stage3_sprint
echo ============================================
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.stage3_sprint'"
if errorlevel 1 echo [WARN] Stage3 exit code: %errorlevel%

echo.
echo ============================================
echo   Stage 4/6: stage4
echo ============================================
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 -m minimal_fsm_demo.stage4'"
if errorlevel 1 echo [WARN] Stage4 exit code: %errorlevel%

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
echo   All 6 stages completed!
echo ============================================
pause
