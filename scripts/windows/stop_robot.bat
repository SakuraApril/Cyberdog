@echo off
title Stop Robot - Damper
echo Killing all stage scripts...
wsl -u root -e bash -lc "docker exec cyberdog_sim sh -c 'pkill -9 -f stage || true'" 2>nul
echo Sending damper...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && python3 /tmp/stop_dog.py'" 2>nul
echo Done.
pause
