#Requires -Version 5.1
# ASCII-safe setup script for Windows PowerShell 5.1

$ErrorActionPreference = 'Stop'

function Write-Step([string]$msg)  { Write-Host "`n==> $msg" -ForegroundColor Cyan }
function Write-Ok([string]$msg)    { Write-Host "OK  $msg" -ForegroundColor Green }
function Write-Warn([string]$msg)  { Write-Host "WARN $msg" -ForegroundColor Yellow }

function Ask([string]$prompt, [string]$default = '') {
    if ($default) {
        $r = Read-Host "$prompt [$default]"
        if ([string]::IsNullOrWhiteSpace($r)) { return $default }
        return $r.Trim()
    }
    return (Read-Host $prompt).Trim()
}

function Ask-YesNo([string]$prompt, [bool]$defaultYes = $true) {
    $hint = if ($defaultYes) { 'Y/n' } else { 'y/N' }
    $r = (Read-Host "$prompt ($hint)").Trim().ToLower()
    if ([string]::IsNullOrWhiteSpace($r)) { return $defaultYes }
    return $r -in @('y', 'yes')
}

function Test-Command([string]$name) {
    return [bool](Get-Command $name -ErrorAction SilentlyContinue)
}

function Refresh-Path {
    $machine = [System.Environment]::GetEnvironmentVariable('Path', 'Machine')
    $user = [System.Environment]::GetEnvironmentVariable('Path', 'User')
    $env:Path = $machine + ';' + $user
}

function Invoke-GitQuiet {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Args)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'SilentlyContinue'
    try {
        & git @Args 2>&1 | Out-Null
        return $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
}

function Install-Winget([string]$id, [string]$label) {
    if (-not (Test-Command 'winget')) {
        Write-Warn "winget not found - install '$label' manually: $id"
        return $false
    }
    Write-Host "Installing $label via winget..."
    winget install --id $id -e --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) {
        Write-Warn "winget install failed for $label"
        return $false
    }
    Write-Ok "$label installed"
    return $true
}

function Find-Python([string]$version) {
    $tag = $version.Replace('.', '')
    $candidates = @(
        "$env:LOCALAPPDATA\Programs\Python\Python$tag\python.exe",
        "$env:ProgramFiles\Python$tag\python.exe",
        "${env:ProgramFiles(x86)}\Python$tag\python.exe",
        "C:\Python$tag\python.exe"
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { return $c }
    }

    # py.exe writes launcher hints to stderr; avoid terminating on NativeCommandError.
    if (Test-Command 'py') {
        $prev = $ErrorActionPreference
        $ErrorActionPreference = 'SilentlyContinue'
        try {
            $out = cmd /c "py -$version -c `"import sys; print(sys.executable)`" 2>nul"
            if ($LASTEXITCODE -eq 0 -and $out) {
                $line = ($out -split "`r?`n" | Where-Object { $_ -match '\\python\.exe$' } | Select-Object -First 1)
                if ($line) { return $line.Trim() }
            }
        } finally {
            $ErrorActionPreference = $prev
        }
    }
    return $null
}

function Ensure-Python([string]$version) {
    $exe = Find-Python $version
    if ($exe) {
        Write-Ok "Python $version found: $exe"
        return $exe
    }
    if (-not (Ask-YesNo "Python $version not found. Install it now with winget?" $true)) {
        throw "Python $version is required."
    }
    $pkg = switch ($version) {
        '3.8'  { 'Python.Python.3.8' }
        '3.12' { 'Python.Python.3.12' }
        default { throw "Unsupported Python version $version" }
    }
    Install-Winget $pkg "Python $version" | Out-Null
    Refresh-Path
    $exe = Find-Python $version
    if (-not $exe) {
        Write-Warn "Python $version not visible yet; waiting for installer to finish registering..."
        Start-Sleep -Seconds 5
        Refresh-Path
        $exe = Find-Python $version
    }
    if (-not $exe) { throw "Python $version still not found after install. Restart PowerShell and re-run setup-local.ps1." }
    Write-Ok "Python $version found: $exe"
    return $exe
}

function Find-CMake {
    if (Test-Command 'cmake') { return 'cmake' }
    $candidates = @(
        "$env:ProgramFiles\CMake\bin\cmake.exe",
        "${env:ProgramFiles(x86)}\CMake\bin\cmake.exe"
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { return $c }
    }
    return $null
}

function Use-CMakeOnPath([string]$cmakePath) {
    if ($cmakePath -and $cmakePath -ne 'cmake') {
        $bin = Split-Path $cmakePath -Parent
        if ($env:Path -notlike "*$bin*") { $env:Path = "$bin;$env:Path" }
    }
}

function Find-OpenSim([string]$hintPath) {
    $candidates = @(
        $hintPath,
        'C:\OpenSim 4.5',
        'C:\Program Files\OpenSim 4.5',
        "$env:ProgramFiles\OpenSim 4.5",
        "$env:LOCALAPPDATA\OpenSim 4.5"
    ) | Where-Object { $_ } | Select-Object -Unique
    foreach ($d in $candidates) {
        if (Test-Path (Join-Path $d 'bin\OpenSim64.exe')) { return $d }
        if (Test-Path (Join-Path $d 'sdk\Python\opensim')) { return $d }
    }
    return $null
}

function Ensure-OpenSim {
    $script:OpenSimDir = Find-OpenSim $script:OpenSimDir
    if ($script:OpenSimDir) {
        Write-Ok "OpenSim: $script:OpenSimDir"
        return
    }

    Write-Warn 'OpenSim 4.5 not found on this PC.'
    Write-Host 'Install OpenSim 4.5 from https://opensim.stanford.edu/downloads/default.html'
    Write-Host 'Typical install folder: C:\OpenSim 4.5'
    Start-Process 'https://opensim.stanford.edu/downloads/default.html'

    while (-not $script:OpenSimDir) {
        $entered = Ask 'After installing OpenSim, enter its install folder' 'C:\OpenSim 4.5'
        $found = Find-OpenSim $entered
        if ($found) {
            $script:OpenSimDir = $found
            break
        }

        Write-Warn "OpenSim not found at '$entered'."
        Write-Host 'Expected: bin\OpenSim64.exe or sdk\Python\opensim under that folder.'
        if (-not (Ask-YesNo 'Try another folder?' $true)) {
            if (Ask-YesNo 'Continue setup without OpenSim for now? (install later; live viewer will not work yet)' $false) {
                $script:OpenSimDir = $entered
                $script:OpenSimMissing = $true
                Write-Warn "Continuing without verified OpenSim. Expected path: $script:OpenSimDir"
                return
            }
            throw "OpenSim not found at '$entered'. Install OpenSim 4.5, then re-run setup-local.ps1."
        }
    }
    Write-Ok "OpenSim: $script:OpenSimDir"
}

function Ensure-Prerequisites {
    Write-Step 'Checking / installing prerequisites'

    if (-not (Test-Command 'git')) {
        if (Ask-YesNo 'Git not found. Install with winget?' $true) {
            Install-Winget 'Git.Git' 'Git' | Out-Null
            Refresh-Path
        } else { throw 'Git is required.' }
    } else { Write-Ok 'Git' }

    $cmake = Find-CMake
    if (-not $cmake) {
        if (Ask-YesNo 'CMake not found. Install with winget?' $true) {
            Install-Winget 'Kitware.CMake' 'CMake' | Out-Null
            Refresh-Path
            $cmake = Find-CMake
        } else { throw 'CMake is required to build Open Ephys.' }
    }
    if (-not $cmake) {
        Write-Warn 'CMake still not found after winget (install may have failed or PATH not refreshed).'
        Write-Host 'Install manually: https://cmake.org/download/  (add CMake to PATH)'
        if (-not (Ask-YesNo 'Press Y after CMake is installed, or to retry detection' $true)) {
            throw 'CMake is required to build Open Ephys.'
        }
        Refresh-Path
        $cmake = Find-CMake
        if (-not $cmake) { throw 'CMake still not found. Restart PowerShell and re-run setup-local.ps1.' }
    }
    Use-CMakeOnPath $cmake
    Write-Ok 'CMake'

    $vsWhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $hasVS = $false
    if (Test-Path $vsWhere) {
        $vsPath = & $vsWhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
        $hasVS = [bool]$vsPath
    }
    if (-not $hasVS) {
        Write-Warn 'Visual Studio 2022 with C++ tools not detected.'
        if (Ask-YesNo 'Install VS 2022 Build Tools + C++ workload? (large download, 2-6 GB)' $true) {
            if (Test-Command 'winget') {
                Write-Host 'This may take 15-30+ minutes...'
                winget install Microsoft.VisualStudio.2022.BuildTools -e --accept-package-agreements --accept-source-agreements --override '--wait --passive --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended'
            } else {
                Write-Warn 'Install manually: https://visualstudio.microsoft.com/downloads/'
                if (-not (Ask-YesNo 'Press Y after you have installed VS Build Tools')) { throw 'VS C++ tools required.' }
            }
        } else { throw 'Visual Studio 2022 C++ tools are required.' }
    } else { Write-Ok 'Visual Studio C++ tools' }

    $script:Py38 = Ensure-Python '3.8'
    $script:Py312 = Ensure-Python '3.12'

    Ensure-OpenSim

    if (-not $script:OpenSimMissing) {
        $setupPyDir = Join-Path $script:OpenSimDir 'sdk\Python'
        $setupPy38 = Join-Path $setupPyDir 'setup_win_python38.py'
        if (Test-Path $setupPy38) {
            Write-Host 'Running OpenSim Python 3.8 setup...'
            Push-Location $setupPyDir
            try {
                & $script:Py38 $setupPy38
                if ($LASTEXITCODE -eq 0) { Write-Ok 'OpenSim Python 3.8 bindings' }
                else { Write-Warn 'OpenSim Python 3.8 setup returned an error; continuing setup.' }
            } finally {
                Pop-Location
            }
        }
    } else {
        Write-Warn 'Skipped OpenSim Python setup (OpenSim not installed yet).'
    }
}

function Get-GitRemoteUrl([string]$dir) {
    if (-not (Test-Path (Join-Path $dir '.git'))) { return $null }
    Push-Location $dir
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'SilentlyContinue'
    $url = git remote get-url origin 2>$null
    $ErrorActionPreference = $prev
    Pop-Location
    return $url
}

function Ensure-GuiRepo([string]$dir, [bool]$update) {
    $url = 'https://github.com/open-ephys/plugin-GUI.git'
    $remote = Get-GitRemoteUrl $dir
    if ($remote -and $remote -notmatch 'plugin-GUI') {
        Write-Warn 'Replacing archived GUI clone with open-ephys/plugin-GUI'
        Remove-Item -Recurse -Force $dir
    }
    Ensure-Clone $url $dir 'main' $update
}

function Ensure-AcquisitionBoardRepo([string]$dir, [bool]$update) {
    $url = 'https://github.com/open-ephys-plugins/acquisition-board.git'
    $hasPluginSources = Test-Path (Join-Path $dir 'Source\DeviceEditor.cpp')
    if ((Test-Path $dir) -and -not $hasPluginSources) {
        Write-Warn 'Replacing hardware acquisition-board clone with open-ephys-plugins/acquisition-board'
        Remove-Item -Recurse -Force $dir
    }
    Ensure-Clone $url $dir 'main' $update
}

function Resolve-AcquisitionPluginSource([string]$acqDir) {
    $source = Join-Path $acqDir 'Source'
    if (-not (Test-Path (Join-Path $source 'DeviceEditor.cpp'))) {
        throw "Cannot find acquisition-board plugin sources under $source"
    }
    New-Item -ItemType Directory -Force -Path (Join-Path $source 'devices\redpitaya') | Out-Null
    return $source
}

function Ensure-Clone([string]$url, [string]$dir, [string]$branch = $null, [bool]$update = $false) {
    if (-not (Test-Path $dir)) {
        if ($branch) { git clone --branch $branch --single-branch $url $dir }
        else         { git clone $url $dir }
        Write-Ok "Cloned $dir"
    } elseif ($update) {
        Push-Location $dir
        Invoke-GitQuiet fetch origin | Out-Null
        if ($branch) {
            Invoke-GitQuiet checkout $branch | Out-Null
            Invoke-GitQuiet pull origin $branch | Out-Null
        } else {
            Invoke-GitQuiet pull | Out-Null
        }
        Pop-Location
        Write-Ok "Updated $dir"
    } else {
        Write-Host "Already exists: $dir (skipping clone; answer Y to pull latest)"
    }
}

function Patch-File([string]$path, [hashtable]$replacements) {
    if (-not (Test-Path $path)) { return }
    $text = Get-Content $path -Raw
    foreach ($k in $replacements.Keys) {
        $text = $text.Replace($k, $replacements[$k])
    }
    Set-Content $path $text -NoNewline -Encoding UTF8
}

Write-Host ''
Write-Host '============================================================' -ForegroundColor Cyan
Write-Host '  Open Ephys + OpenSim Plugin - Interactive Local Setup' -ForegroundColor Cyan
Write-Host '============================================================' -ForegroundColor Cyan
Write-Host ''

Write-Step 'Configuration questions'

$script:WorkDir    = Ask 'OpenSim work folder (models + Python scripts)' "$env:USERPROFILE\Open-Sim--Bio-Mech"
$script:DevRoot    = Ask 'Development root (GUI + plugins cloned here)' "$env:USERPROFILE\dev"
$script:OpenSimDir = Ask 'OpenSim install folder (leave default if not installed yet)' 'C:\OpenSim 4.5'
$script:BuildConfig = Ask 'Open Ephys build config (Debug or Release)' 'Debug'
if ($script:BuildConfig -notin @('Debug', 'Release')) { $script:BuildConfig = 'Debug' }

Write-Host ''
Write-Host 'Hardware type:'
Write-Host '  1 = ESP32-S3 over USB (serial bridge)'
Write-Host '  2 = ESP32-S3 over Wi-Fi'
Write-Host '  3 = Red Pitaya'
$hw = Ask 'Choose hardware' '1'
switch ($hw) {
    '2' { $script:HwMode = 'esp32-wifi'; $script:Esp32Host = Ask 'ESP32 IP address or hostname' '192.168.1.100' }
    '3' { $script:HwMode = 'redpitaya' }
    default { $script:HwMode = 'esp32-usb'; $script:Esp32Com = Ask 'ESP32 USB COM port' 'COM5' }
}

$script:Esp32RecordDir = Ask 'ESP32 CSV recording folder' "$env:USERPROFILE\Documents\Arduino\ESP32-S3-1\results"
$script:PluginBranch   = Ask 'Plugin git branch' 'main'
$updateRepos           = Ask-YesNo 'Pull latest code if repos already exist?' $true
$runBuild              = Ask-YesNo 'Build open-ephys.exe after setup? (needs VS + CMake)' $true

Write-Host ''
Write-Host 'Summary:'
Write-Host "  Work dir:    $script:WorkDir"
Write-Host "  Dev root:    $script:DevRoot"
Write-Host "  OpenSim:     $script:OpenSimDir"
Write-Host "  Hardware:    $script:HwMode"
if ($script:HwMode -eq 'esp32-usb') { Write-Host "  ESP32 COM:   $script:Esp32Com" }
if ($script:HwMode -eq 'esp32-wifi') { Write-Host "  ESP32 host:  $script:Esp32Host" }
Write-Host "  Build:       $script:BuildConfig"
if (-not (Ask-YesNo 'Continue with setup?' $true)) { exit 0 }

$script:OpenSimMissing = $false
Ensure-Prerequisites

Write-Step 'Creating folders'
New-Item -ItemType Directory -Force -Path $script:WorkDir, $script:DevRoot, $script:Esp32RecordDir | Out-Null
Write-Ok 'Folders ready'

Write-Step 'Cloning repositories'
$pluginDir = Join-Path $script:DevRoot 'Plugin'
$guiDir    = Join-Path $script:DevRoot 'GUI'
$acqDir    = Join-Path $script:DevRoot 'acquisition-board'
$esp32Dir  = Join-Path $script:DevRoot 'ESP32-S3'

Ensure-Clone 'https://github.com/Minkeejung0415/Plugin.git' $pluginDir $script:PluginBranch $updateRepos
Ensure-GuiRepo $guiDir $updateRepos
if (Test-Path (Join-Path $guiDir '.git')) {
    Push-Location $guiDir
    Invoke-GitQuiet submodule update --init --recursive | Out-Null
    Pop-Location
}
Ensure-AcquisitionBoardRepo $acqDir $updateRepos
if ($script:HwMode -like 'esp32*') {
    Ensure-Clone 'https://github.com/Minkeejung0415/ESP32-S3.git' $esp32Dir $null $updateRepos
}

Write-Step 'Installing plugin sources into acquisition-board'
$acqPlugin = Resolve-AcquisitionPluginSource $acqDir

Copy-Item -Force (Join-Path $pluginDir 'Acqboard.h')          (Join-Path $acqPlugin 'devices\AcquisitionBoard.h')
Copy-Item -Force (Join-Path $pluginDir 'acqboard.ccp')        (Join-Path $acqPlugin 'devices\redpitaya\AcqBoardRedPitaya.cpp')
Copy-Item -Force (Join-Path $pluginDir 'Acqboardredpitaya.h') (Join-Path $acqPlugin 'devices\redpitaya\AcqBoardRedPitaya.h')
Copy-Item -Force (Join-Path $pluginDir 'device editor.cpp')   (Join-Path $acqPlugin 'DeviceEditor.cpp')
Copy-Item -Force (Join-Path $pluginDir 'device editor.h')     (Join-Path $acqPlugin 'DeviceEditor.h')
Copy-Item -Force (Join-Path $pluginDir 'devicethread.cpp')    (Join-Path $acqPlugin 'DeviceThread.cpp')
Write-Ok 'Plugin sources copied (including RedPitaya AcquisitionBoard.h patch)'

Write-Step 'Patching paths for your machine'
$workEsc  = $script:WorkDir.Replace('\', '\\')
$esp32Esc = $script:Esp32RecordDir.Replace('\', '\\')

$cppFile = Join-Path $acqPlugin 'devices\redpitaya\AcqBoardRedPitaya.cpp'
if (Test-Path $cppFile) {
    (Get-Content $cppFile -Raw) `
        -replace 'C:\\Users\\KIN Student\\Open-Sim--Bio-Mech', $workEsc `
        -replace 'C:\\Users\\KIN Student\\Documents\\Arduino\\ESP32-S3-1\\results', $esp32Esc |
        Set-Content $cppFile -NoNewline -Encoding UTF8
}

$workFiles = @(
    'Rajagopal2015_opensense_calibrated.osim', 'Rajagopal2015_opensense.osim',
    'ephys_imuIK_Setup.xml', '_neutral_frame.sto',
    'opensim_live_realtime.py', 'ephys_to_opensim_bridge.py',
    'opensim_sensor_map.json', 'opensim_display_joint.json',
    'run_bridge.bat', 'diagnose_oe_udp.py', 'test_udp_sender.py'
)
foreach ($f in $workFiles) {
    $src = Join-Path $pluginDir $f
    if (Test-Path $src) { Copy-Item -Force $src (Join-Path $script:WorkDir $f) }
}

foreach ($py in @('opensim_live_realtime.py', 'ephys_to_opensim_bridge.py')) {
    Patch-File (Join-Path $script:WorkDir $py) @{
        'C:\Users\KIN Student\Open-Sim--Bio-Mech' = $script:WorkDir
        'C:\OpenSim 4.5' = $script:OpenSimDir
    }
}
Write-Ok "Work folder: $script:WorkDir"

Write-Step 'Installing Python packages (numpy, imufusion)'
& $script:Py38  -m pip install --upgrade pip numpy imufusion
& $script:Py312 -m pip install --upgrade pip numpy imufusion
Write-Ok 'Python packages'

$buildDir = Join-Path $guiDir 'Build'
$oeExe = Join-Path $buildDir "$($script:BuildConfig)\open-ephys.exe"
$pluginDll = Join-Path $buildDir "$($script:BuildConfig)\plugins\acquisition-board.dll"
if ($runBuild) {
    Write-Step "Building Open Ephys GUI ($($script:BuildConfig))"
    New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
    Push-Location $buildDir
    if (-not (Test-Path 'CMakeCache.txt')) {
        cmake .. -G 'Visual Studio 17 2022' -A x64
    }
    cmake --build . --config $script:BuildConfig --target open-ephys
    Pop-Location
    if (Test-Path $oeExe) { Write-Ok "Built: $oeExe" }
    else { Write-Warn 'open-ephys.exe not found - check GUI build output above' }

    Write-Step "Building acquisition-board plugin ($($script:BuildConfig))"
    $acqBuildDir = Join-Path $acqDir 'Build'
    New-Item -ItemType Directory -Force -Path $acqBuildDir | Out-Null
    $guiBase = (Resolve-Path $guiDir).Path
    Push-Location $acqBuildDir
    if (-not (Test-Path 'CMakeCache.txt')) {
        cmake .. -G 'Visual Studio 17 2022' -A x64 "-DGUI_BASE_DIR=$guiBase"
    }
    cmake --build . --config $script:BuildConfig
    Pop-Location
    if (Test-Path $pluginDll) { Write-Ok "Built plugin: $pluginDll" }
    else { Write-Warn 'acquisition-board.dll not found - check plugin build output above' }
} else {
    Write-Warn "Skipped build (build GUI from $buildDir, plugin from $(Join-Path $acqDir 'Build'))"
}

Write-Step 'Writing launch scripts'
$startLiveBat = Join-Path $script:WorkDir 'start_opensim_live.bat'
$batLines = @(
    '@echo off',
    "cd /d `"$($script:WorkDir)`"",
    "set PATH=$($script:OpenSimDir)\bin;%PATH%",
    "set PYTHONPATH=$($script:OpenSimDir)\sdk\Python;%PYTHONPATH%",
    'set OPENSIM_LIVE_SOURCE=real_redpitaya',
    'start "OpenSim Live" cmd /k py -3.8 -u opensim_live_realtime.py'
) -join "`r`n"
Set-Content $startLiveBat $batLines -Encoding ASCII

$startOe = Join-Path $script:DevRoot 'start-open-ephys.ps1'
$oeLines = @(
    '$ErrorActionPreference = ''Stop''',
    "`$exe = '$oeExe'",
    'if (-not (Test-Path $exe)) { throw "Not found: $exe - run setup with build enabled" }',
    'Start-Process $exe'
) -join "`r`n"
Set-Content $startOe $oeLines -Encoding UTF8

if ($script:HwMode -eq 'esp32-usb') {
    $startBridge = Join-Path $script:DevRoot 'start-esp32-bridge.ps1'
    $brLines = @(
        '$ErrorActionPreference = ''Stop''',
        "& '$esp32Dir\host\run_usb_plugin_bridge.ps1' '$($script:Esp32Com)'"
    ) -join "`r`n"
    Set-Content $startBridge $brLines -Encoding UTF8
}

$configPath = Join-Path $script:DevRoot 'setup-config.json'
@{
    work_dir = $script:WorkDir
    dev_root = $script:DevRoot
    opensim_dir = $script:OpenSimDir
    build_config = $script:BuildConfig
    hw_mode = $script:HwMode
    esp32_com = $(if ($script:Esp32Com) { $script:Esp32Com } else { '' })
    esp32_host = $(if ($script:Esp32Host) { $script:Esp32Host } else { '' })
    plugin_branch = $script:PluginBranch
} | ConvertTo-Json | Set-Content $configPath -Encoding UTF8

if ($script:HwMode -like 'esp32*') {
    Write-Step 'ESP32 firmware (manual step)'
    Write-Host "Flash firmware from: $esp32Dir"
    if (Ask-YesNo 'Open ESP32-S3 folder in Explorer now?' $false) {
        Start-Process explorer.exe $esp32Dir
    }
}

Write-Host ''
Write-Host '============================================================' -ForegroundColor Green
Write-Host '  SETUP COMPLETE' -ForegroundColor Green
Write-Host '============================================================' -ForegroundColor Green
Write-Host ''
Write-Host "Saved config: $configPath"
Write-Host ''
Write-Host 'Daily workflow:' -ForegroundColor Yellow
if ($script:HwMode -eq 'esp32-usb') {
    Write-Host "  1. powershell -File `"$($script:DevRoot)\start-esp32-bridge.ps1`""
}
if ($script:HwMode -eq 'esp32-wifi') {
    Write-Host "  1. Set Node IP in plugin to: $($script:Esp32Host)"
}
if ($script:HwMode -eq 'redpitaya') {
    Write-Host '  1. Power on Red Pitaya'
}
Write-Host "  2. powershell -File `"$startOe`""
Write-Host '  3. In GUI: Acquisition Board -> Display Joint -> OpenSim Live -> Play'
Write-Host '  4. Turn Filter ON (ESP32) so quaternions stream to OpenSim'
Write-Host ''
Write-Host "Manual OpenSim Live: $startLiveBat"
Write-Host ''
