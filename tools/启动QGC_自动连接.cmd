@echo off
REM ============================================================
REM  Launch QGroundControl with automatic flight-controller
REM  serial-port detection.
REM
REM  Fixes: "cannot connect after unplugging / rebooting the FC".
REM  It finds the FC's CURRENT COM port by USB VID/PID, kills any
REM  stale QGC process, patches QGroundControl.ini, then launches
REM  QGC and verifies that parameters actually downloaded.
REM
REM  NOTE: keep this file pure ASCII. cmd.exe parses batch files in
REM        the OEM code page, so UTF-8 Chinese comments break it.
REM ============================================================
setlocal
set "PS1=%~dp0qgc_auto_connect.ps1"
if not exist "%PS1%" (
    echo [ERROR] qgc_auto_connect.ps1 not found next to this .cmd
    echo         Expected: "%PS1%"
    pause
    exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%PS1%"
exit /b %errorlevel%
