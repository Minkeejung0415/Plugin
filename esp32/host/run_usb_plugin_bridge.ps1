# USB serial -> TCP :5000 for Open Ephys Plugin Acq Board (Minkeejung0415/Plugin).
# Requires: pip install pyserial; board flashed v1.4+ with USB_OPEN_EPHYS_MODE true; close Serial Monitor.
#
# Usage:
#   .\host\run_usb_plugin_bridge.ps1 COM5
#
# Alternate (no Plugin C++ changes): python host\rp_compat_gateway.py COM5
#   after adding plugin-patches\hosts.txt to C:\Windows\System32\drivers\etc\hosts

param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$ComPort
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

Write-Host "Bridge: $ComPort -> 127.0.0.1:5000 (--plugin). Open Ephys: Acq Board, Node IP 127.0.0.1"
python host\serial_tcp_bridge.py $ComPort --plugin
