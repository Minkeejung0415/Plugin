# setup-local.ps1 — one-shot local integration for Open Ephys + OpenSim plugin
# Run in PowerShell (as your user, not necessarily admin):
#   Set-ExecutionPolicy -Scope Process Bypass; .\setup-local.ps1
#
# Edit these paths for YOUR machine before running:

$WORK_DIR        = "$env:USERPROFILE\Open-Sim--Bio-Mech"
$OPENSIM_DIR     = "C:\OpenSim 4.5"
$DEV_ROOT        = "$env:USERPROFILE\dev"          # where GUI + plugins are cloned
$ESP32_COM       = "COM5"                         # USB serial port for ESP32 bridge
$ESP32_RECORD_DIR = "$env:USERPROFILE\Documents\Arduino\ESP32-S3-1\results"
$PLUGIN_BRANCH   = "cursor/opensim-target-angle-display-ad4a"
$BUILD_CONFIG    = "Debug"                        # Debug or Release — use ONE everywhere

$GUI_REPO        = "https://github.com/open-ephys/GUI.git"
$ACQ_REPO        = "https://github.com/open-ephys/acquisition-board.git"
$PLUGIN_REPO     = "https://github.com/Minkeejung0415/Plugin.git"
$ESP32_REPO      = "https://github.com/Minkeejung0415/ESP32-S3.git"

$ErrorActionPreference = "Stop"

Write-Host "=== Open Ephys + OpenSim local setup ===" -ForegroundColor Cyan
Write-Host "WORK_DIR: $WORK_DIR"
Write-Host "OPENSIM:  $OPENSIM_DIR"
Write-Host "DEV_ROOT: $DEV_ROOT"
Write-Host ""

# --- 1. Folders ---
New-Item -ItemType Directory -Force -Path $WORK_DIR, $DEV_ROOT | Out-Null

# --- 2. Clone repos ---
function Ensure-Clone($url, $dir, $branch = $null) {
    if (-not (Test-Path $dir)) {
        if ($branch) { git clone --branch $branch --single-branch $url $dir }
        else         { git clone $url $dir }
    } else {
        Write-Host "Already cloned: $dir"
    }
}

$pluginDir = Join-Path $DEV_ROOT "Plugin"
$guiDir    = Join-Path $DEV_ROOT "GUI"
$acqDir    = Join-Path $DEV_ROOT "acquisition-board"
$esp32Dir  = Join-Path $DEV_ROOT "ESP32-S3"

Ensure-Clone $PLUGIN_REPO $pluginDir $PLUGIN_BRANCH
Ensure-Clone $GUI_REPO    $guiDir
Ensure-Clone $ACQ_REPO    $acqDir
Ensure-Clone $ESP32_REPO  $esp32Dir

# --- 3. Copy plugin sources into acquisition-board ---
$acqPlugin = Join-Path $acqDir "Source\Plugin"
if (-not (Test-Path $acqPlugin)) { throw "Not found: $acqPlugin — check acquisition-board layout" }

Copy-Item -Force (Join-Path $pluginDir "acqboard.ccp")              (Join-Path $acqPlugin "devices\redpitaya\AcqBoardRedPitaya.cpp")
Copy-Item -Force (Join-Path $pluginDir "Acqboardredpitaya.h")       (Join-Path $acqPlugin "devices\redpitaya\AcqBoardRedPitaya.h")
Copy-Item -Force (Join-Path $pluginDir "device editor.cpp")         (Join-Path $acqPlugin "DeviceEditor.cpp")
Copy-Item -Force (Join-Path $pluginDir "device editor.h")            (Join-Path $acqPlugin "DeviceEditor.h")
Copy-Item -Force (Join-Path $pluginDir "devicethread.cpp")          (Join-Path $acqPlugin "DeviceThread.cpp")

Write-Host "Plugin sources copied into acquisition-board." -ForegroundColor Green

# --- 4. Patch hardcoded paths in plugin + Python ---
$workEsc   = $WORK_DIR -replace '\\', '\\'
$esp32Esc  = $ESP32_RECORD_DIR -replace '\\', '\\'
$opensimEsc = $OPENSIM_DIR -replace '\\', '\\'

$cppFile = Join-Path $acqPlugin "devices\redpitaya\AcqBoardRedPitaya.cpp"
(Get-Content $cppFile -Raw) `
    -replace 'C:\\Users\\KIN Student\\Open-Sim--Bio-Mech', $workEsc `
    -replace 'C:\\Users\\KIN Student\\Documents\\Arduino\\ESP32-S3-1\\results', $esp32Esc |
    Set-Content $cppFile -NoNewline

# --- 5. Copy OpenSim work files ---
$workFiles = @(
    "Rajagopal2015_opensense_calibrated.osim",
    "Rajagopal2015_opensense.osim",
    "ephys_imuIK_Setup.xml",
    "_neutral_frame.sto",
    "opensim_live_realtime.py",
    "ephys_to_opensim_bridge.py",
    "opensim_sensor_map.json",
    "opensim_display_joint.json",
    "run_bridge.bat",
    "diagnose_oe_udp.py",
    "test_udp_sender.py"
)
foreach ($f in $workFiles) {
    $src = Join-Path $pluginDir $f
    if (Test-Path $src) { Copy-Item -Force $src (Join-Path $WORK_DIR $f) }
}

# Patch Python paths
$pyFiles = @("opensim_live_realtime.py", "ephys_to_opensim_bridge.py")
foreach ($py in $pyFiles) {
    $p = Join-Path $WORK_DIR $py
    if (-not (Test-Path $p)) { continue }
    (Get-Content $p -Raw) `
        -replace 'C:\\Users\\KIN Student\\Open-Sim--Bio-Mech', ($WORK_DIR -replace '\\', '\\') `
        -replace 'C:\\OpenSim 4\.5', ($OPENSIM_DIR -replace '\\', '\\') |
        Set-Content $p -NoNewline
}

Write-Host "OpenSim work dir ready: $WORK_DIR" -ForegroundColor Green

# --- 6. Python packages ---
Write-Host "Installing Python packages..."
py -3.8  -m pip install --upgrade pip numpy imufusion 2>$null; if ($LASTEXITCODE -ne 0) { python -m pip install --upgrade pip numpy imufusion }
py -3.12 -m pip install --upgrade pip numpy imufusion 2>$null

# --- 7. Link acquisition-board into GUI plugins folder ---
$guiPlugins = Join-Path $guiDir "Build\plugins"
New-Item -ItemType Directory -Force -Path $guiPlugins | Out-Null
$acqLink = Join-Path $guiPlugins "acquisition-board"
if (Test-Path $acqLink) { Remove-Item -Recurse -Force $acqLink }
# Junction/symlink so GUI build picks up the plugin
cmd /c mklink /J "$acqLink" "$acqDir" | Out-Null
Write-Host "Linked acquisition-board -> GUI Build\plugins" -ForegroundColor Green

# --- 8. Build Open Ephys (requires Visual Studio 2022 + CMake) ---
$buildDir = Join-Path $guiDir "Build"
if (-not (Test-Path $buildDir)) { New-Item -ItemType Directory -Force -Path $buildDir | Out-Null }

Push-Location $buildDir
if (-not (Test-Path "CMakeCache.txt")) {
    Write-Host "Running CMake configure..."
    cmake .. -G "Visual Studio 17 2022" -A x64
}
Write-Host "Building Open Ephys ($BUILD_CONFIG) — this may take several minutes..."
cmake --build . --config $BUILD_CONFIG --target open-ephys
Pop-Location

$oeExe = Join-Path $buildDir "$BUILD_CONFIG\open-ephys.exe"
if (Test-Path $oeExe) {
    Write-Host "Build OK: $oeExe" -ForegroundColor Green
} else {
    Write-Host "WARN: open-ephys.exe not found — build may have failed. Check VS/CMake." -ForegroundColor Yellow
}

# --- 9. Write quick-launch helper scripts ---
@"

@echo off
cd /d "$WORK_DIR"
set PATH=$OPENSIM_DIR\bin;%PATH%
set PYTHONPATH=$OPENSIM_DIR\sdk\Python;%PYTHONPATH%
set OPENSIM_LIVE_SOURCE=real_redpitaya
start "OpenSim Live" cmd /k py -3.8 -u opensim_live_realtime.py
"@ | Set-Content (Join-Path $WORK_DIR "start_opensim_live.bat") -Encoding ASCII

@"

`$ErrorActionPreference = 'Stop'
& "$esp32Dir\host\run_usb_plugin_bridge.ps1" $ESP32_COM
"@ | Set-Content (Join-Path $DEV_ROOT "start-esp32-bridge.ps1") -Encoding UTF8

@"

`$ErrorActionPreference = 'Stop'
Start-Process "$oeExe"
"@ | Set-Content (Join-Path $DEV_ROOT "start-open-ephys.ps1") -Encoding UTF8

Write-Host ""
Write-Host "=== DONE ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "Daily workflow (3 terminals):" -ForegroundColor Yellow
Write-Host "  1. ESP32 USB:  powershell -File `"$DEV_ROOT\start-esp32-bridge.ps1`""
Write-Host "  2. Open Ephys: powershell -File `"$DEV_ROOT\start-open-ephys.ps1`""
Write-Host "     -> Add Acquisition Board -> set Display Joint -> OpenSim Live -> Play"
Write-Host "  3. (auto)      OpenSim Live opens when you click the button in the plugin"
Write-Host ""
Write-Host "Or run OpenSim Live manually:"
Write-Host "  $WORK_DIR\start_opensim_live.bat"
Write-Host ""
