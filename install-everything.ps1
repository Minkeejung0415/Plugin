# Bootstrap: download this script + setup-local.ps1, then run interactive setup.
# Paste this ENTIRE block into PowerShell:

Set-ExecutionPolicy -Scope Process Bypass -Force

$DEV_ROOT = "$env:USERPROFILE\dev"
$PLUGIN_DIR = Join-Path $DEV_ROOT "Plugin"
New-Item -ItemType Directory -Force -Path $DEV_ROOT | Out-Null

if (-not (Test-Path (Join-Path $PLUGIN_DIR "setup-local.ps1"))) {
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        Write-Host "Git not found. Installing via winget..."
        winget install Git.Git -e --accept-package-agreements --accept-source-agreements
        $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                    [System.Environment]::GetEnvironmentVariable("Path", "User")
    }
    git clone --branch cursor/opensim-target-angle-display-ad4a --single-branch `
        https://github.com/Minkeejung0415/Plugin.git $PLUGIN_DIR
}

& (Join-Path $PLUGIN_DIR "setup-local.ps1")
