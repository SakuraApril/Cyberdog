@echo off
setlocal EnableExtensions EnableDelayedExpansion

if "%~1"=="" (
  echo Usage: common_sync.bat ^<SRC_DIR_LINUX^> ^<DST_DIR_LINUX^> [HASH_FILE_LINUX]
  exit /b 2
)
if "%~2"=="" (
  echo Usage: common_sync.bat ^<SRC_DIR_LINUX^> ^<DST_DIR_LINUX^> [HASH_FILE_LINUX]
  exit /b 2
)

set "SRC_DIR=%~1"
set "DST_DIR=%~2"
set "HASH_FILE=%~3"
if "%HASH_FILE%"=="" set "HASH_FILE=/tmp/minimal_fsm_demo_sync.sha256"

echo [SYNC] Starting container...
wsl -u root -e bash -lc "docker start cyberdog_sim >/dev/null"
if errorlevel 1 (
  echo [SYNC][ERROR] Container start failed.
  exit /b 1
)

echo [SYNC] Checking control script changes...
set "SYNC_STATE="
for /f "delims=" %%I in ('wsl -u root -e bash -lc "set -e; H=$(find \"%SRC_DIR%\" -maxdepth 1 -type f -name '*.py' -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'); OLD=$(docker exec cyberdog_sim sh -c 'cat %HASH_FILE% || true'); if [ \"$H\" = \"$OLD\" ] && [ -n \"$H\" ]; then echo SKIP; else echo SYNC; fi"') do set "SYNC_STATE=%%I"

if /I "%SYNC_STATE%"=="SYNC" (
  echo [SYNC] Changes detected, syncing files...
  wsl -u root -e bash -lc "set -e; docker cp %SRC_DIR%/. cyberdog_sim:%DST_DIR%/; H=$(find \"%SRC_DIR%\" -maxdepth 1 -type f -name '*.py' -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'); docker exec cyberdog_sim bash -lc \"echo $H > %HASH_FILE%\""
  if errorlevel 1 (
    echo [SYNC][ERROR] Sync failed.
    exit /b 1
  )
  echo [SYNC] Updating installed python package files...
  wsl -u root -e bash -lc "docker exec cyberdog_sim bash -lc 'if [ -d /home/cyberdog_sim/install/lib/python3.8/site-packages/minimal_fsm_demo ]; then cp /home/cyberdog_sim/src/cyberdog_simulator/minimal_fsm_demo/minimal_fsm_demo/*.py /home/cyberdog_sim/install/lib/python3.8/site-packages/minimal_fsm_demo/; fi'"
  echo [SYNC] Sync done.
) else (
  echo [SYNC] No control script changes, skip sync.
)

exit /b 0
