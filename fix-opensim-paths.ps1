#Requires -Version 5.1
# Re-apply OpenSim / ESP32 paths and rebuild acquisition-board.dll
param(
    [string]$WorkDir = "$env:USERPROFILE\Documents\Plugin",
    [string]$DevRoot = "$env:USERPROFILE\dev",
    [string]$OpenSimDir = 'C:\OpenSim 4.5',
    [string]$Esp32RecordDir = "$env:USERPROFILE\Documents\Arduino\ESP32-S3-1\results",
    [string]$BuildConfig = 'Debug',
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'

function Write-Step([string]$msg)  { Write-Host "`n==> $msg" -ForegroundColor Cyan }
function Write-Ok([string]$msg)    { Write-Host "OK  $msg" -ForegroundColor Green }
function Write-Warn([string]$msg)  { Write-Host "WARN $msg" -ForegroundColor Yellow }

function Patch-AcquisitionBoardCpp([string]$cppFile, [string]$workDir, [string]$esp32RecordDir) {
    $workEsc = $workDir.Replace('\', '\\')
    $esp32Esc = $esp32RecordDir.Replace('\', '\\')
    $text = Get-Content $cppFile -Raw
    $text = $text -replace 'const char\* kOpenSimWorkDir\s*=\s*"[^"]*";', "const char* kOpenSimWorkDir = `"$workEsc`";"
    $text = $text -replace 'const char\* kEsp32RecordDir\s*=\s*"[^"]*";', "const char* kEsp32RecordDir = `"$esp32Esc`";"
    Set-Content $cppFile $text -Encoding UTF8
}

function Patch-OpenSimPython([string]$pyFile, [string]$workDir, [string]$openSimDir) {
    $modelPath = Join-Path $workDir 'Rajagopal2015_opensense_calibrated.osim'
    $text = Get-Content $pyFile -Raw
    $text = $text -replace 'WORK_DIR\s*=\s*r"[^"]*"', "WORK_DIR = r`"$workDir`""
    $text = $text -replace 'MODEL_PATH\s*=\s*r"[^"]*"', "MODEL_PATH = r`"$modelPath`""
    $text = $text.Replace('C:\Users\KIN Student\Open-Sim--Bio-Mech', $workDir)
    $text = $text.Replace('C:\OpenSim 4.5', $openSimDir)
    Set-Content $pyFile $text -Encoding UTF8
}

function Show-OpenSimPathAudit([string]$workDir, [string]$devRoot) {
    Write-Host ''
    Write-Host 'OpenSim path audit:' -ForegroundColor Yellow
    $cpp = Join-Path $devRoot 'acquisition-board\Source\devices\redpitaya\AcqBoardRedPitaya.cpp'
    if (Test-Path $cpp) {
        Select-String -Path $cpp -Pattern 'kOpenSimWorkDir\s*=|kEsp32RecordDir\s*=' |
            ForEach-Object { Write-Host "  C++  $($_.Line.Trim())" }
    }
    foreach ($name in @('opensim_live_realtime.py', 'ephys_to_opensim_bridge.py')) {
        $py = Join-Path $workDir $name
        if (Test-Path $py) {
            Select-String -Path $py -Pattern '^(MODEL_PATH|WORK_DIR)\s*=' |
                ForEach-Object { Write-Host "  PY   $($_.Line.Trim())" }
        } else {
            Write-Warn "  PY   missing: $py"
        }
    }
    $dll = Join-Path $devRoot "GUI\Build\$BuildConfig\plugins\acquisition-board.dll"
    if (Test-Path $dll) {
        $item = Get-Item $dll
        Write-Host "  DLL  $($item.FullName) (updated $($item.LastWriteTime))"
    } else {
        Write-Warn "  DLL  missing: $dll"
    }
}

Write-Host ''
Write-Host 'Fix OpenSim paths + rebuild acquisition-board plugin' -ForegroundColor Cyan
Write-Host "  Work dir:  $WorkDir"
Write-Host "  Dev root:  $DevRoot"
Write-Host ''

if (-not (Test-Path $WorkDir)) { throw "Work dir not found: $WorkDir" }

$cppFile = Join-Path $DevRoot 'acquisition-board\Source\devices\redpitaya\AcqBoardRedPitaya.cpp'
if (-not (Test-Path $cppFile)) { throw "Plugin source not found: $cppFile" }

Write-Step 'Patching C++ plugin source'
Patch-AcquisitionBoardCpp $cppFile $WorkDir $Esp32RecordDir
Write-Ok 'AcqBoardRedPitaya.cpp'

Write-Step 'Patching Python scripts in work dir'
foreach ($name in @('opensim_live_realtime.py', 'ephys_to_opensim_bridge.py')) {
    $py = Join-Path $WorkDir $name
    if (Test-Path $py) {
        Patch-OpenSimPython $py $WorkDir $OpenSimDir
        Write-Ok $name
    } else {
        Write-Warn "missing: $py"
    }
}

if (-not $SkipBuild) {
    Write-Step "Building acquisition-board ($BuildConfig)"
    $acqBuild = Join-Path $DevRoot 'acquisition-board\Build'
    if (-not (Test-Path $acqBuild)) { throw "Build folder missing: $acqBuild" }
    Push-Location $acqBuild
    cmake --build . --config $BuildConfig
    cmake --install . --config $BuildConfig
    Pop-Location
    Write-Ok 'acquisition-board.dll installed to GUI plugins folder'
}

Show-OpenSimPathAudit $WorkDir $DevRoot

Write-Host ''
Write-Host 'Done. Close and restart Open Ephys, then try OpenSim Live again.' -ForegroundColor Green
Write-Host ''
