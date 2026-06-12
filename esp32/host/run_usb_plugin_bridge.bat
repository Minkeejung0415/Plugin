@echo off
setlocal
if "%~1"=="" (
  echo Usage: host\run_usb_plugin_bridge.bat COM5
  echo   Board: USB_OPEN_EPHYS_MODE true. Close Serial Monitor first.
  exit /b 1
)
cd /d "%~dp0\.."
echo Bridge: %1 -^> 127.0.0.1:5000 (--plugin). Open Ephys: Acq Board, Node IP 127.0.0.1, 100 Hz.
python host\serial_tcp_bridge.py %1 --plugin
