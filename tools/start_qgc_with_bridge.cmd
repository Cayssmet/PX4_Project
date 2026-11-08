@echo off
setlocal enabledelayedexpansion
REM ===========================================================================
REM  One-click launcher: flight-controller USB serial -> UDP bridge + QGC
REM ---------------------------------------------------------------------------
REM  Why this exists:
REM    QGC 5.0.8 on this Windows box has a bug: it opens COM9 exclusively but
REM    reads nothing (no tlog written, no parameter download). While it holds
REM    the port, every other program gets:
REM        Access to the path 'COM9' is denied.
REM    Bridging the raw flight-controller bytes to UDP 14550 lets QGC connect
REM    over a completely different code path, which has been reliable.
REM
REM  Usage:
REM    Double-click. A minimised bridge window opens (shows bidi traffic stats).
REM    Closing that window stops the bridge and QGC will disconnect.
REM
REM  Optional argument: the COM port, default COM9.
REM      start_qgc_with_bridge.cmd COM7
REM
REM  Prerequisite (pyserial inside tools\.pylibs, PYTHONPATH is set here):
REM    python -m pip install --target tools\.pylibs pyserial
REM
REM  NOTE: keep this file pure ASCII. cmd.exe parses batch files in the OEM
REM        codepage; UTF-8 Chinese comments break parsing on this machine.
REM ===========================================================================

set "HERE=%~dp0"
set "PYTHONPATH=%HERE%.pylibs"
set "BRIDGE=%HERE%mavlink_usb_udp_bridge.py"
REM Launch the QGC binary directly instead of the Chinese-named .cmd wrapper
REM next to the repo: a non-ASCII path inside a batch file is a codepage trap.
set "QGC_EXE=%HERE%..\..\QGroundControl\bin\QGroundControl.exe"
set "COMPORT=%~1"
if "%COMPORT%"=="" set "COMPORT=COM9"
if "%PYTHON%"=="" set "PYTHON=python"

if not exist "%BRIDGE%" goto :no_bridge
if defined UAV_DRY_RUN goto :dry_run

echo [1/2] Starting bridge: %COMPORT% -^> udp://127.0.0.1:14550
start "MAVLink USB-UDP bridge" /min "%PYTHON%" -u "%BRIDGE%" %COMPORT% 115200 127.0.0.1 14550 14551

REM Give the bridge 2 seconds to take the COM port first, otherwise QGC may
REM grab the serial port on startup and fall back into the broken code path.
timeout /t 2 /nobreak >nul

set "QT_QUICK_BACKEND=software"
echo [2/2] Starting QGroundControl
if not exist "%QGC_EXE%" goto :no_qgc
start "" "%QGC_EXE%"
echo.
echo Done. Two rules, otherwise QGC will steal the COM port again:
echo   1. QGC - Application Settings - General:
echo        "Pixhawk"  auto-connect  MUST stay UNCHECKED
echo        "UDP"      auto-connect  MUST stay CHECKED
echo   2. Never connect to the COM port from QGC's Comm Links page.
echo.
exit /b 0

:no_bridge
echo [ERROR] bridge script not found: %BRIDGE%
pause
exit /b 1

:no_qgc
echo [WARN] not found: %QGC_EXE%
echo [HINT] expected QGC at D:\09_PX4\QGroundControl\bin\QGroundControl.exe
echo        the bridge is still running, start QGC manually.
exit /b 1

:dry_run
echo [dry-run] PYTHONPATH = %PYTHONPATH%
echo [dry-run] PYTHON     = %PYTHON%
echo [dry-run] BRIDGE     = %BRIDGE%
echo [dry-run] COMPORT    = %COMPORT%
echo [dry-run] QGC_EXE    = %QGC_EXE%
if exist "%QGC_EXE%" (echo [dry-run] QGC found = yes) else (echo [dry-run] QGC found = no)
if exist "%BRIDGE%"  (echo [dry-run] bridge found = yes) else (echo [dry-run] bridge found = no)
exit /b 0
