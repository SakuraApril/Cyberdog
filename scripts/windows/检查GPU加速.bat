@echo off
setlocal EnableExtensions
title Cyberdog GPU Check
echo ============================================
echo   Cyberdog GPU / WSLg Check
echo ============================================
echo.

echo [1/4] Ensure container is running...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"

echo [2/4] Check GPU in container...
wsl -u root -e bash -lc "docker exec -e DISPLAY=:0 cyberdog_sim bash -lc 'nvidia-smi || true'"
echo.

echo [3/4] Check WSLg display env in container...
wsl -u root -e bash -lc "docker exec -e DISPLAY=:0 -e QT_QPA_PLATFORM=xcb -e LD_LIBRARY_PATH=/usr/lib/wsl/lib:/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/lib/x86_64-linux-gnu:/usr/lib:/lib cyberdog_sim bash -lc 'echo DISPLAY=$DISPLAY; echo QT_QPA_PLATFORM=$QT_QPA_PLATFORM; echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH; ls -l /mnt/wslg/.X11-unix/X0 2>/dev/null || true'"
echo.

echo [4/4] Check Gazebo pose channel...
wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'gz model -m robot -p 2>&1 || true'"
echo.

echo [DONE] If nvidia-smi and DISPLAY show normally, GPU path is basically ready.
pause

