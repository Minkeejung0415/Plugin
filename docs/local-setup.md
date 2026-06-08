# Local machine setup (interactive)

## One command — paste into PowerShell

This downloads the setup script, **asks you questions**, **installs missing tools**, and builds everything:

```powershell
Set-ExecutionPolicy -Scope Process Bypass -Force; iwr -useb https://raw.githubusercontent.com/Minkeejung0415/Plugin/cursor/opensim-target-angle-display-ad4a/install-everything.ps1 | iex
```

If that URL is unavailable, clone manually then run:

```powershell
Set-ExecutionPolicy -Scope Process Bypass -Force
git clone --branch cursor/opensim-target-angle-display-ad4a --single-branch https://github.com/Minkeejung0415/Plugin.git "$env:USERPROFILE\dev\Plugin"
& "$env:USERPROFILE\dev\Plugin\setup-local.ps1"
```

## Questions you will be asked

| Question | Default | Purpose |
|----------|---------|---------|
| OpenSim work folder | `%USERPROFILE%\Open-Sim--Bio-Mech` | Models + Python scripts |
| Development root | `%USERPROFILE%\dev` | GUI + plugin clones |
| OpenSim install folder | `C:\OpenSim 4.5` | OpenSim 4.5 location |
| Build config | `Debug` | Open Ephys build type |
| Hardware type | `1` ESP32 USB | USB / Wi-Fi / Red Pitaya |
| COM port or IP | `COM5` or your IP | Hardware connection |
| Pull latest repos? | Yes | Update existing clones |
| Build open-ephys? | Yes | Compile the GUI |

## Repos cloned by setup

| Folder | Repository |
|--------|------------|
| `dev\GUI` | [open-ephys/plugin-GUI](https://github.com/open-ephys/plugin-GUI) (not the archived `open-ephys/GUI`) |
| `dev\acquisition-board` | [open-ephys-plugins/acquisition-board](https://github.com/open-ephys-plugins/acquisition-board) (not the PCB hardware repo) |

If a previous run cloned the wrong repos, setup replaces them automatically.

## What gets installed automatically (via winget)

- Git
- CMake
- Python 3.8
- Python 3.12
- Visual Studio 2022 Build Tools + C++ workload (if missing)
- `pip install numpy imufusion` for both Python versions

## What you install manually (script opens the page)

- **OpenSim 4.5** — [opensim.stanford.edu/downloads](https://opensim.stanford.edu/downloads/default.html). Install to `C:\OpenSim 4.5`, then re-enter that path when the script asks. Do not press Enter until the installer has finished.
- **ESP32 firmware** — flash from the cloned `ESP32-S3` repo (Arduino IDE)

If CMake winget install fails, install from [cmake.org/download](https://cmake.org/download/) and choose **Add CMake to PATH**, then press Y when the script asks to retry detection.

## After setup — daily use

**ESP32 USB:**

```powershell
powershell -File "$env:USERPROFILE\dev\start-esp32-bridge.ps1"
powershell -File "$env:USERPROFILE\dev\start-open-ephys.ps1"
```

In Open Ephys: **Display Joint** → **OpenSim Live** → **Play** → **Filter ON**
