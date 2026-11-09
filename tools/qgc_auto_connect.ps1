param([switch]$NoPause)

# ============================================================
#  qgc_auto_connect.ps1
#
#  Fixes: "QGroundControl cannot connect after unplugging the
#  flight controller / power-cycling it / restarting QGC".
#
#  It does four things:
#    1. Find the FC's CURRENT COM port by USB VID/PID
#       (never assume it is always COM9).
#    2. Kill any stale/hung QGroundControl process and wait for
#       the COM port to be really released.
#    3. Write that port into QGroundControl.ini (link "FC-USB").
#    4. Launch QGC and verify that parameters actually downloaded.
#       "Port opened but nothing read" is a known QGC glitch, so
#       it retries once automatically.
#
#  NOTE: this file is intentionally pure ASCII on purpose.
#        Windows PowerShell 5.1 reads .ps1 files as the OEM code
#        page (GBK here) unless they carry a UTF-8 BOM; any edit
#        that strips the BOM turns non-ASCII text into garbage and
#        breaks parsing. Keep it ASCII and it can never break.
#
#  Usage: double-click the .cmd file next to this script.
# ============================================================

$ErrorActionPreference = "Continue"

$ini      = "$env:APPDATA\QGroundControl\QGroundControl.ini"
$cache    = "$env:APPDATA\QGroundControl\ParamCache\1_1.v2"
# Launch QGroundControl.exe DIRECTLY. Do not go through a .cmd in a path that
# contains non-ASCII characters: this file is pure ASCII, so Windows
# PowerShell 5.1 decodes any Chinese path bytes as GBK and the path is garbage
# ("The system cannot find the file specified"). The env var below is exactly
# what the original launcher .cmd did (software rendering, avoids a white screen).
$qgcExe   = "D:\09_PX4\QGroundControl\bin\QGroundControl.exe"
$qgcDir   = "D:\09_PX4\QGroundControl\bin"

function Say($m) { Write-Host $m }
function Pause-Here {
    if ($NoPause) { return }
    Say ""
    Say "Press any key to close this window..."
    try { $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown") } catch { }
}

# ---- 1) locate the flight controller COM port ---------------------------
# A MicoAir743v2 shows up under two possible USB identities:
#   VID_1B8C&PID_0036 = PX4 application firmware  (normal, connectable)
#   VID_3162&PID_004B = MicoAir own VID (usually the bootloader: PX4 not running)
function Find-FcPort {
    $found = @()
    Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue |
        Where-Object { $_.Status -eq 'OK' -and $_.InstanceId -match 'VID_(1B8C&PID_0036|3162&PID_004B)' } |
        ForEach-Object {
            $name = (Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName 'DEVPKEY_Device_FriendlyName' -ErrorAction SilentlyContinue).Data
            if ($name -match '(COM\d+)') {
                $found += [pscustomobject]@{
                    Port         = $Matches[1]
                    InstanceId   = $_.InstanceId
                    IsBootloader = ($_.InstanceId -match 'VID_3162')
                }
            }
        }
    return $found
}

function Test-PortFree($port) {
    try {
        $sp = New-Object System.IO.Ports.SerialPort $port, 115200
        $sp.Open(); $sp.Close(); return $true
    } catch { return $false }
}

# ---- 2) kill stale QGC --------------------------------------------------
function Stop-QGC {
    $q = Get-Process QGroundControl -ErrorAction SilentlyContinue
    if ($q) {
        foreach ($p in $q) { Say ("  Killing stale QGC process PID {0}" -f $p.Id); Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
        Start-Sleep -Seconds 2
        return $true
    }
    return $false
}

# ---- 3) write the port into QGroundControl.ini --------------------------
function Set-LinkPort($port) {
    if (-not (Test-Path $ini)) { Say "  x QGroundControl.ini not found"; return $false }
    Copy-Item $ini "$ini.autofix" -Force
    $lines = Get-Content $ini
    $out = New-Object System.Collections.Generic.List[string]
    $inLink = $false; $seen = $false
    foreach ($l in $lines) {
        if ($l -match '^\[LinkConfigurations\]') {
            $out.Add($l)
            $out.Add('count=1')
            $out.Add('Link0\name=FC-USB')
            $out.Add('Link0\type=0')
            $out.Add('Link0\auto=true')
            $out.Add('Link0\high_latency=false')
            $out.Add('Link0\baud=115200')
            $out.Add('Link0\dataBits=8')
            $out.Add('Link0\flowControl=0')
            $out.Add('Link0\stopBits=1')
            $out.Add('Link0\parity=0')
            # Escaping must match exactly what QGC/QSettings writes:
            # real port name is \\.\COM<n>; QSettings escapes every
            # backslash again => the file must contain
            #   4 backslashes + dot + 2 backslashes + COM<n>
            # One missing backslash makes the port unopenable and QGC
            # silently reports "not connected" with no error message.
            $out.Add('Link0\portName=\\\\.\\' + $port)
            $out.Add('Link0\portDisplayName=' + $port)
            $inLink = $true; $seen = $true
            continue
        }
        if ($inLink) { if ($l -match '^\[') { $inLink = $false } else { continue } }
        if ($l -match '^autoConnectUDP=')     { $out.Add('autoConnectUDP=false');     continue }
        if ($l -match '^autoConnectPixhawk=') { $out.Add('autoConnectPixhawk=false'); continue }
        $out.Add($l)
    }
    if (-not $seen) { Say "  x no [LinkConfigurations] section in the ini"; return $false }

    # UTF-8 WITHOUT BOM on purpose: QGC parses this with QSettings.
    [System.IO.File]::WriteAllLines($ini, $out, (New-Object System.Text.UTF8Encoding $false))

    # Read back and verify. A wrong port name fails silently in QGC.
    $check = ([System.IO.File]::ReadAllText($ini)) -split "`r?`n" | Where-Object { $_ -match '^Link0\\portName=' } | Select-Object -First 1
    if (-not $check -or $check -notmatch [regex]::Escape($port)) {
        Say ("  x port-name write verification FAILED, ini contains: {0}" -f $check)
        Say "    QGC would silently fail to connect; aborting."
        return $false
    }
    Say ("  OK link port set to {0} (read-back verified)" -f $port)
    return $true
}

# ---- 4) launch and verify ----------------------------------------------
function Start-AndVerify($port) {
    $before = $null
    if (Test-Path $cache) { $before = (Get-Item $cache).LastWriteTime }
    Say "  Launching QGC ..."
    $env:QT_QUICK_BACKEND = 'software'
    Start-Process -FilePath $qgcExe -WorkingDirectory $qgcDir -WindowStyle Minimized
    $t0 = Get-Date
    for ($i = 0; $i -lt 45; $i++) {
        Start-Sleep -Seconds 2
        if (Test-PortFree $port) { continue }   # QGC has not opened it yet
        if (Test-Path $cache) {
            $now = (Get-Item $cache).LastWriteTime
            if (-not $before -or $now -gt $before) {
                Say ("  OK connected for real at {0:HH:mm:ss} (parameters downloaded, {1:N0} s)" -f (Get-Date), ((Get-Date)-$t0).TotalSeconds)
                return $true
            }
        }
        if ($i -eq 20) { Say "  ... 40 s and no parameter download yet, still waiting" }
    }
    return $false
}

# ======================== main ==========================================
Say "==============================================="
Say " QGC auto-connect (auto COM detection + verify)"
Say "==============================================="
Say ""

$ports = Find-FcPort
if ($ports.Count -eq 0) {
    Say "x Flight controller NOT found. Check:"
    Say "  1) USB cable seated (prefer a rear USB-A port, no hub / front panel)"
    Say "  2) Flight controller powered (USB power is enough)"
    Say "  3) Device Manager -> Ports (COM & LPT) shows 'MicoAir743v2'"
    Pause-Here; exit 1
}

$fc = $ports | Where-Object { -not $_.IsBootloader } | Select-Object -First 1
if (-not $fc) {
    $bl = $ports[0]
    Say ("! FC found on {0} but its USB identity is VID_3162 (MicoAir bootloader)." -f $bl.Port)
    Say "  That means the PX4 firmware is NOT running, so QGC can never connect."
    Say "  Fix: unplug and replug USB so the FC boots normally."
    Say "  (Only if it keeps landing in the bootloader do you need to reflash.)"
    Pause-Here; exit 2
}

Say ("OK flight controller found on {0}  ({1})" -f $fc.Port, $fc.InstanceId)
Say ""

if (Stop-QGC) { Say "  Waiting for the COM port to be released ..." }
$freed = $false
for ($i = 0; $i -lt 40; $i++) {
    if (Test-PortFree $fc.Port) { $freed = $true; break }
    Start-Sleep -Milliseconds 500
}
if (-not $freed) { Say ("  ! {0} is still busy; trying anyway ..." -f $fc.Port) }

if (-not (Set-LinkPort $fc.Port)) { Pause-Here; exit 3 }
Say ""

$ok = Start-AndVerify $fc.Port

if (-not $ok) {
    Say ""
    Say "  ! First attempt failed (QGC opened the port but read nothing -"
    Say "    a known QGC glitch). Retrying once ..."
    Stop-QGC | Out-Null
    for ($i = 0; $i -lt 40; $i++) { if (Test-PortFree $fc.Port) { break }; Start-Sleep -Milliseconds 500 }
    $ok = Start-AndVerify $fc.Port
}

Say ""
if ($ok) {
    Say "==============================================="
    Say " Connected."
    Say ""
    Say " * Wait a full 60 seconds before opening the Actuators page!"
    Say "   Opening it too early wipes the page permanently and the"
    Say "   only cure is restarting QGC again."
    Say "==============================================="
} else {
    Say "==============================================="
    Say " Both attempts failed. Try:"
    Say "  1) unplug USB -> use a different USB-A port -> rerun this script"
    Say "  2) try another cable (some cables are charge-only, no data wires)"
    Say "  3) in Device Manager, uninstall all greyed-out COM devices, replug"
    Say "==============================================="
}
Pause-Here
