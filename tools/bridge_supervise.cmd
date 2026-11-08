@echo off
setlocal
REM ===========================================================================
REM  Supervisor for mavlink_usb_udp_bridge.py
REM ---------------------------------------------------------------------------
REM  Why this exists:
REM    A crash inside the bridge used to kill it silently. QGC talks UDP, so it
REM    never notices the bridge died: the vehicle just stops updating and the
REM    user sees "QGC disconnected again" with the COM port held by nothing.
REM    This loop restarts the bridge within ~2 seconds after any exit.
REM
REM  Also important: never leave a second program holding COM9. Anything that
REM  opens the port (probe scripts, QGC Comm Links, other terminals) breaks the
REM  bridge and therefore QGC. See docs/QGC manual, troubleshooting section.
REM
REM  Usage: bridge_supervise.cmd [COM9]
REM         Normally started minimised by start_qgc_with_bridge.cmd.
REM
REM  Prerequisite (pyserial inside tools\.pylibs, PYTHONPATH set here):
REM    python -m pip install --target tools\.pylibs pyserial
REM
REM  NOTE: keep this file pure ASCII. cmd.exe parses batch files in the OEM
REM        codepage; UTF-8 Chinese comments break parsing on this machine.
REM ===========================================================================

set "HERE=%~dp0"
set "PYTHONPATH=%HERE%.pylibs"
set "BRIDGE=%HERE%mavlink_usb_udp_bridge.py"
set "COMPORT=%~1"
if "%COMPORT%"=="" set "COMPORT=COM9"
if not "%~2"=="" set "BAUD=%~2"
if "%BAUD%"=="" set "BAUD=115200"
if "%PYTHON%"=="" set "PYTHON=python"

if not exist "%BRIDGE%" goto :no_bridge

echo [supervise] bridge script : %BRIDGE%
echo [supervise] serial port   : %COMPORT% @ %BAUD%
echo [supervise] udp target    : 127.0.0.1:14550 (local bind 14551)
echo [supervise] ctrl-c or close this window to stop
echo.

:loop
echo [supervise] starting bridge at %DATE% %TIME%
"%PYTHON%" -u "%BRIDGE%" %COMPORT% %BAUD% 127.0.0.1 14550 14551
set "RC=%ERRORLEVEL%"
echo.
echo [supervise] bridge exited with code %RC%, restarting in 2s
echo [supervise] (if this repeats forever, check that no other program holds %COMPORT%)
timeout /t 2 /nobreak >nul
goto loop

:no_bridge
echo [ERROR] bridge script not found: %BRIDGE%
pause
exit /b 1
