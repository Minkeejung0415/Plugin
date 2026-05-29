@echo off
cd /d "%~dp0"
echo =====================================================
echo  Open Ephys ^<-^> OpenSim Bridge Launcher
echo =====================================================
echo.

REM Check if Python is available
python --version 2>nul
if %errorlevel% neq 0 (
    echo ERROR: Python not found. Make sure Python is installed and in PATH.
    pause
    exit /b 1
)

REM Check required packages
echo Checking required packages...
python -c "import imufusion" 2>nul
if %errorlevel% neq 0 (
    echo.
    echo ERROR: 'imufusion' is not installed.
    echo Run this command to fix it:
    echo   pip install imufusion
    echo   or: py -3.12 -m pip install imufusion
    echo.
    pause
    exit /b 1
)

python -c "import numpy" 2>nul
if %errorlevel% neq 0 (
    echo ERROR: 'numpy' is not installed.  Run: pip install numpy
    pause
    exit /b 1
)

echo All packages OK.
echo.
echo Choose a mode:
echo   1. listen (default - wait 3 seconds for UDP data)
echo   2. listen --realtime (live 3D skeleton window)
echo   3. listen --until-idle 5 (stop after 5s of no data)
echo   4. offline (process imu_sample_data.csv)
echo.
set /p CHOICE="Enter 1/2/3/4 (or press Enter for default): "

if "%CHOICE%"=="2" (
    python ephys_to_opensim_bridge.py listen --realtime
) else if "%CHOICE%"=="3" (
    python ephys_to_opensim_bridge.py listen --until-idle 5
) else if "%CHOICE%"=="4" (
    python ephys_to_opensim_bridge.py offline
) else (
    python ephys_to_opensim_bridge.py listen
)

echo.
echo =====================================================
echo  Script finished.
echo =====================================================
pause
