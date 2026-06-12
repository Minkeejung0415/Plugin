# Red Pitaya OpenSim Plugin — Full Repository Explanation
### (Written for someone who has never touched this code before)

---

## The Big Idea (Start Here)

Imagine strapping little motion-detector stickers on someone's legs and hips. Each sticker is a tiny computer chip called an **IMU** (Inertial Measurement Unit). It's basically the same chip inside a smartphone that knows when you tilt the phone.

The stickers send their data to a small credit-card-sized computer called a **Red Pitaya**, which does math to figure out "which way is each sticker pointing right now?" That "pointing direction" gets turned into four special numbers called a **quaternion**. Think of a quaternion as GPS coordinates for *direction* instead of location.

Those numbers get sent over a network cable to a Windows PC. A piece of software called **Open Ephys** records the numbers. Another piece of software called **OpenSim** uses the numbers to animate a 3D skeleton of a human body in real-time — like a puppet controlled by the motion stickers.

The cool new thing this project adds: when a **trigger** fires (like a starting-gun signal from the experiment), the operator can choose to see *only a few selected joint angles* (e.g. "just show me the right knee angle") displayed right next to the skeleton's clock timer.

---

## The Cast of Characters (File Map)

```
Red Pitaya Hardware (C firmware)
├── RedPitaya_justin.c        ← runs ON the Red Pitaya board itself
├── axi_header.h              ← register addresses for the FPGA hardware
│
Sensor Fusion Math (C library — used by the firmware)
├── vqf.h / vqf.c             ← the fancy math that converts raw sensor data → quaternion
├── sensor_fusion.h / .c      ← wrapper: manage multiple IMU chips at once
│
Open Ephys Plugin (C++/JUCE — runs on the Windows PC)
├── Acqboard.h                ← abstract "what every acquisition board must do"
├── devices/redpitaya/
│   └── AcqBoardRedPitaya.h   ← the Red Pitaya-specific implementation
├── device editor.h / .cpp    ← the control panel the operator sees on screen
├── devicethread.cpp          ← detects and initialises the board at startup
│
Python Bridge (Python 3.8 — runs on the Windows PC)
├── ephys_to_opensim_bridge.py  ← batch mode: collect data → run IK → open GUI
├── opensim_live_realtime.py    ← live mode: stream data → animate skeleton in real-time
│
Shared Definitions
├── opensim_joint_catalog.h   ← C++ list of the 7 joints the UI can display
├── opensim_joint_catalog.py  ← Python copy of the same list
│
Configuration Files (written at runtime)
├── opensim_sensor_map.json         ← which sensor slot maps to which body part
├── opensim_display_joint.json      ← which single joint the HUD dropdown shows
├── opensim_joint_display_config.json ← which joints to show after a trigger fires
│
Tests
├── tests/vqf_decimation_ts_test.c         ← checks the VQF timing math
└── tests/redpitaya_stream_framing_test.c  ← checks the binary packet parser
```

---

## File-by-File Deep Dive

---

### 1. `vqf.h` and `vqf.c` — The Math Brain

**What it does in one sentence:** Takes raw numbers from an IMU chip (accelerometer + gyroscope + optional magnetometer) and continuously figures out "which way is this chip currently pointing?"

**Simple analogy:** Imagine you close your eyes and someone spins you in a chair. You can *feel* yourself turning (that is your gyroscope). But after a while your sense of direction drifts — you might think you are facing north when you are actually facing east. To fix this, you open your eyes and look at the floor (that is your accelerometer, checking where gravity pulls). If you have a compass, you can also check that (magnetometer). VQF does all three corrections automatically.

**Key data structures:**

| Name | What it stores | Kid-friendly analogy |
|------|---------------|----------------------|
| `VQFParams` | Tuning knobs (how fast to trust accel vs gyro) | Like bass/treble knobs on a stereo |
| `VQFState` | Everything the filter remembers between samples | Like your brain's short-term memory |
| `VQFCoefficients` | Pre-computed math constants (computed once at startup) | Like the multiplication tables you memorized |
| `VQF` | One complete filter instance (params + state + coeffs) | One complete "direction-tracking brain" |

**Key functions:**

| Function | What it does |
|----------|-------------|
| `vqf_init()` | Create a brand-new filter, knowing how fast the sensor takes measurements |
| `vqf_update_gyr()` | Feed it a new gyroscope reading; it updates the "spin integral" |
| `vqf_update_acc()` | Feed it a new accelerometer reading; corrects tilt drift using gravity |
| `vqf_update_mag()` | Feed it a new magnetometer reading; corrects compass heading drift |
| `vqf_update()` | Convenience: feed gyro + accel together (6D mode, no compass) |
| `vqf_update_9d()` | Convenience: feed gyro + accel + magnetometer together (9D mode) |
| `vqf_get_quat_6d()` | Ask: "which way am I pointing right now?" (returns 4 numbers) |
| `vqf_get_bias_estimate()` | Ask: "how much is the gyro lying to me right now?" |
| `vqf_get_rest_detected()` | Ask: "has the sensor been sitting still long enough?" |
| `vqf_get_mag_dist_detected()` | Ask: "is something magnetic messing up the compass?" |

**The core algorithm (step by step):**

1. **Gyro step** (`vqf_update_gyr`): Every time-tick, ask the gyroscope "how many degrees per second are you spinning?" Multiply by the time step to get "how many degrees did you rotate?", then apply that rotation to the current orientation quaternion. This is like dead-reckoning navigation — very accurate for short times but slowly drifts.

2. **Accel step** (`vqf_update_acc`): The accelerometer, when not moving, always points toward Earth's center. Compare where the filter *thinks* "down" is versus where the accelerometer says "down" is. If they disagree, gently correct the quaternion. A slow low-pass filter (3-second time constant by default) is used so that genuine motion accelerations don't confuse the correction.

3. **Mag step** (`vqf_update_mag`): The magnetometer points toward magnetic north. Use it to fix the "yaw" (left-right spin) drift that the gyro+accel alone can't catch. Automatically detect if a metal object or motor is messing up the magnetic field and ignore the magnetometer when that happens.

4. **Bias estimation**: Slowly learn how much the gyroscope drifts on its own (every gyro chip has a small constant error). When the sensor sits still, estimate that drift very precisely. During motion, estimate it less accurately but still track it.

---

### 2. `sensor_fusion.h` and `sensor_fusion.c` — The Fusion Manager

**What it does in one sentence:** Manages a whole team of VQF filters — one per IMU chip — and converts raw integer sensor readings into usable quaternion numbers.

**Simple analogy:** Imagine you have 8 motion-tracker stickers and each one needs its own "direction-tracking brain" (VQF). This module is the manager who hands each brain its raw data, tells it what units to convert to, and collects the results.

**Why this exists separately from VQF:** VQF only understands physical units (meters/second² for acceleration, radians/second for rotation). But the actual sensor chip spits out raw 16-bit integers. This module converts those integers to physical units and handles the per-chip quirks.

**Key data structures:**

| Name | What it stores |
|------|---------------|
| `FusionSensorConfig` | One sensor's settings: chip type, scale factors, sample rate |
| `FusionSensorState` | One sensor's live state: its VQF filter, last quaternion, status flags |
| `FusionModuleState` | The whole system: array of sensor states + global on/off switch |
| `FusionSensorType` | Enum: which chip model (MPU6050, MPU9250, ICM20948, BNO055, or generic) |
| `FusionStatusFlags` | Bit flags: VALID, ENABLED, REST_DETECTED, MAG_DISTURBANCE, MAGNETOMETER_USED |
| `FusionResult` | Error codes: FUSION_OK, or one of several FUSION_ERR_... values |

**Scale factors — why they matter:**
The sensor chip gives you a number like `4096`. Alone, that is meaningless. The scale factor (e.g. `9.80665 / 16384.0` for a ±2g MPU6050) converts it to real-world meters/second² (i.e. `2.394 m/s²`). Different chips and different range settings use different scale factors:

| Chip | Accel scale | Gyro scale |
|------|------------|-----------|
| MPU6050 | 1g = 16384 counts (default ±2g range) | 1°/s = 131 counts |
| MPU9250 | Same as MPU6050 defaults | Same |
| ICM20948 | Same as MPU6050 defaults | Same |
| BNO055 | 1 m/s² = 100 counts | 1°/s = 16 counts |

**Key functions:**

| Function | What it does |
|----------|-------------|
| `fusion_init(count, rate)` | Allocate memory for `count` sensor slots, set the base sample rate |
| `fusion_register_sensor_ex(i, config)` | Tell slot `i` which chip type it has and its scale factors |
| `fusion_update_sensor(i, raw_acc, raw_gyr, raw_mag, fresh, quat_out)` | Feed one sensor's raw readings, get back a quaternion |
| `fusion_get_status_flags(i)` | Check if sensor `i` is valid, at rest, has mag disturbance, etc. |
| `fusion_shutdown()` | Free all memory and reset to blank state |

**The Q15 format:**
Floating-point quaternion components are in the range [-1.0, +1.0]. To save bandwidth in the hardware stream, they are encoded as 16-bit integers using "Q15 format": multiply by 32767 and round. So `+1.0` becomes `32767`, `-1.0` becomes `-32767`, `0.5` becomes `16384`. The receiving code divides by 32767 to get back floats.

**Decimation handling:**
If the Red Pitaya FPGA runs at 1000 Hz but a specific sensor is only read at 100 Hz (because it is slow), the VQF filter must know the *actual* per-sensor sample rate. Otherwise it would integrate the gyroscope 10× too fast and the angle would grow 10× too large. `imu_sample_rate_hz` in `FusionSensorConfig` fixes this.

---

### 3. `RedPitaya_justin.c` — The Firmware (Runs On The Board)

**What it does in one sentence:** Runs on the Red Pitaya's ARM processor, reads IMU chips via I2C/SPI bus, runs sensor fusion, packages data into binary frames, and streams them over TCP to the Windows PC.

**Simple analogy:** This is the little postman that lives inside the Red Pitaya. It knocks on each IMU chip's door, collects numbers, does the direction-math (VQF), stuffs everything into envelopes, and mails them over the network.

**Key structures defined here:**
- `SensorInstance` — one physical IMU chip: its I2C address, AXI memory-mapped register location, gyro/accel bias corrections, magnetometer cache, and configuration sent from the PC.
- `HardwareContext` — the whole board: up to 6 sensors (4 I2C + 2 SPI), GPIO counter/reset pointers, total channel count.

**The binary stream format (22-byte header + payload):**

Every frame sent over TCP looks like this:

```
Bytes 0-3:   offset       (int32)  — always 0 in current firmware
Bytes 4-7:   bytes_per_frame (int32) — size of the payload that follows
Bytes 8-9:   dtype        (int16)  — always 3 (means "int16 samples")
Bytes 10-13: num_elements (int32)  — always 2 (samples per element)
Bytes 14-17: total_channels (int32) — how many int16 values per sample
Bytes 18-21: sample_count (int32)  — frame sequence number
[payload: total_channels × 2 bytes of int16 sensor values]
```

**Startup sequence:**
1. Map the FPGA's AXI GPIO registers into memory
2. Scan I2C bus for connected IMU chips (MPU9250, ICM20948, etc.)
3. Configure each chip (full-scale ranges, sample rate)
4. Run gyro bias calibration (3000 samples while still)
5. Initialize the sensor fusion module
6. Open a TCP server socket and wait for the Open Ephys plugin to connect
7. Start the main streaming loop

**The main loop:**
Every hardware tick (at the configured Hz rate):
1. Read raw int16 accelerometer/gyro values from each active sensor
2. Call `fusion_update_sensor()` to get updated quaternions
3. Build a binary frame: raw IMU channels first, then qw/qx/qy/qz for each sensor
4. Send the frame over TCP

**Command protocol:**
While streaming, the firmware listens for text commands from the PC:
- `FREQ <hz>` — change the hardware sample rate
- `CFG ACC <sensor> <preset>` — change a sensor's accelerometer full-scale range
- `CFG GYR <sensor> <preset>` — change a sensor's gyroscope full-scale range
- `CFG SRATE <sensor> <hz>` — change a sensor's effective sample rate
- `START` / `STOP` — begin/end streaming

---

### 4. `Acqboard.h` — The Abstract Blueprint

**What it does in one sentence:** Defines the contract that every acquisition board implementation must follow — a C++ interface (abstract class) listing every function that Open Ephys will call.

**Simple analogy:** Think of it as the job description for "Acquisition Board Employee". It says "you must be able to set a sample rate", "you must be able to start streaming", "you must have a gain setting", etc. Any specific board (Red Pitaya, Opal Kelly, ONI) must hire someone who fulfills every requirement.

This file does **not** contain any implementation — only declarations (pure virtual functions). The Red Pitaya implementation lives in `AcqBoardRedPitaya.h`.

---

### 5. `devices/redpitaya/AcqBoardRedPitaya.h` — The Red Pitaya Manager (C++ side)

**What it does in one sentence:** The Windows-side C++ object that connects to the Red Pitaya over the network, receives its binary stream, and feeds samples into the Open Ephys recording engine.

**Simple analogy:** This is the receptionist on the Windows PC side. It calls the Red Pitaya postman (firmware), negotiates a connection, and passes all the received envelopes to the Open Ephys filing system.

**Key methods:**

| Method | What it does |
|--------|-------------|
| `detectBoard()` | Tries to find a Red Pitaya by sending a TCP handshake to `rp-*.local` or an ESP32 node address |
| `initializeBoard()` | Sends initial configuration, sets up channel count |
| `startAcquisition()` / `stopAcquisition()` | Opens/closes the streaming socket, sends START/STOP commands |
| `run()` | The main loop: reads binary frames, parses headers, scales int16 counts to float voltages, feeds Open Ephys |
| `sendSensorCfgAcc/Gyr/Srate()` | Sends live CFG commands to the firmware while streaming |
| `launchOpenSimLive()` | Spawns `opensim_live_realtime.py` as a subprocess |
| `launchOpenSimMotion()` | Spawns `ephys_to_opensim_bridge.py` as a subprocess |
| `writeJointDisplayConfig()` | Atomically writes `opensim_joint_display_config.json` with the current joint selection and a sequence number |
| `setJointDisplaySelected(i, true/false)` | Toggle one joint on/off in the display filter |
| `getSelectedDisplayJoints()` | Return the list of joint names currently selected |

**Joint display persistence:**
The plugin stores the selected joints both in memory and in the Open Ephys XML state file (loaded/saved via `loadJointDisplayFromXml` / `saveJointDisplayToXml`). Every time the selection changes, `writeJointDisplayConfig()` atomically writes the JSON sidecar file that the Python bridge watches.

**The frame parsing loop in `run()`:**
1. Read 22-byte header from socket
2. Resync if header looks corrupt (slide byte-by-byte until a valid header is found)
3. Read `bytes_per_frame` bytes of payload
4. If `bytes_per_frame` changed vs last frame, update channel layout (fusion channels were added/removed)
5. For each channel, scale the raw int16 value by the appropriate preset: `value_float = raw_int16 × scale`
6. Append samples to the Open Ephys buffer

---

### 6. `device editor.h` and `device editor.cpp` — The Control Panel

**What it does in one sentence:** Draws and handles the user interface panel you see inside Open Ephys when a Red Pitaya is connected.

**Simple analogy:** This is the dashboard of the car — it shows knobs, buttons, and labels, and connects them to the engine (AcqBoardRedPitaya).

**The layout — 6 columns:**

| Column | Controls |
|--------|---------|
| 1 | Sample rate label, Record button, OpenSim Live launch button |
| 2 | Hardware filter toggle, Analog In gain, Analog Out voltage |
| 3 | Accelerometer range selector, Gyroscope range selector, Sensor sample-rate selector |
| 4 | Active sensor picker, Body-segment assignment, "Generate Motion" button |
| 5 | Joint display toggle checkboxes (up to 6 joints, color-coded by body segment) |
| 6 | Node IP address text field, Display-Joint dropdown |

**How the joint toggle works:**
Each of the 7 joints in the catalog gets a checkbox. When the user ticks one, the editor calls `board->setJointDisplaySelected(index, true)`, which updates the in-memory set and writes `opensim_joint_display_config.json`. The Python bridge picks up the file change within 50 ms and updates the HUD in the OpenSim window.

---

### 7. `devicethread.cpp` — Board Detection

**What it does in one sentence:** On startup, tries each known board type (OpalKelly, ONI, RedPitaya, Simulated) and selects the first one that responds.

**Simple analogy:** When Open Ephys starts, this code knocks on every possible hardware door until someone answers. If the Red Pitaya answers, it hands control to `AcqBoardRedPitaya`.

---

### 8. `opensim_joint_catalog.h` and `opensim_joint_catalog.py` — The Joint Dictionary

**What they do in one sentence:** Define the master list of 7 joints that the system can display, with their OpenSim coordinate names, short abbreviations, and which body segment's IMU drives them.

**These two files must always stay in sync** — the C++ UI and the Python bridge both use them.

| Coordinate name | Abbreviation | Body segment |
|----------------|--------------|-------------|
| `pelvis_tilt` | `pelvis` / `pelvis_tilt` | `pelvis_imu` |
| `hip_flexion_r` | `hip_r` | `femur_r_imu` |
| `knee_angle_r` | `knee_r` | `tibia_r_imu` |
| `ankle_angle_r` | `ankle_r` | `calcn_r_imu` |
| `hip_flexion_l` | `hip_l` | `femur_l_imu` |
| `knee_angle_l` | `knee_l` | `tibia_l_imu` |
| `ankle_angle_l` | `ankle_l` | `calcn_l_imu` |

**Python functions:**
- `validate_joints(list)` — filters out unknown names, removes duplicates, sorts in catalog order, enforces max 6 joints
- `coordinate_for(name)` — resolves an abbreviation like `"knee_r"` to the full OpenSim name `"knee_angle_r"`
- `abbrev_for(coordinate)` — reverse: full name → abbreviation
- `format_angle_line(abbrev, degrees)` — formats a display line like `"knee_r: 35.2°"`

---

### 9. `opensim_live_realtime.py` — The Live Skeleton Animator

**What it does in one sentence:** Listens for sensor data on UDP port 5000, runs inverse kinematics (IK) to compute joint angles, and displays a moving skeleton in the Simbody 3D window.

**Simple analogy:** This is the movie projector. The Red Pitaya sends film frames (sensor data) over the network; this script projects them onto the screen (OpenSim window) at up to 20 frames per second, while showing selected joint angles as a heads-up display.

**The two threads:**

1. **UDP receiver thread** (`_udp_ahrs_thread`): Runs continuously, receives packets from port 5000, runs AHRS (attitude/heading) fusion to convert accel+gyro into quaternions, and deposits the latest frame into a shared queue.

2. **Main/visualizer thread** (`run_live`): Reads from the queue, calls the OpenSim IK solver, updates the 3D model state, renders the skeleton, and updates the joint-angle HUD.

**Packet formats handled:**
- **v3 (preferred):** `[timestamp, 3.0, N, N×{ax,ay,az,gx,gy,gz}]` — raw accel+gyro, Python does its own AHRS
- **v2 (quaternion):** `[timestamp, 2.0, N, N×{qw,qx,qy,qz}]` — pre-fused quaternions from firmware
- **Legacy:** `[48 floats]` or `[1 timestamp + 48 floats]` — no version tag, assume 8 IMUs

**Calibration (first 3 seconds):**
When a new session starts, the script collects 3 seconds of still-standing data and computes the average accelerometer and gyroscope offset for each sensor. All subsequent samples have this average subtracted. This removes sensor bias (the zero-g offset error that all cheap sensors have).

**The IK loop (how joint angles are computed):**
1. Build a `TimeSeriesTableRotation` from the latest quaternions
2. Create an `OrientationsReference` pointing at that table
3. Call `ikSolver.assemble(state)` — OpenSim minimizes the error between the sensor orientations and the model's predicted orientations by adjusting joint angles
4. Read joint angles from `model.getCoordinateSet().get(name).getValue(state)` (in radians, converted to degrees)

**The HUD display:**
- Tries `DecorativeText.setIsScreenText(True)` first — puts text directly in the 3D viewport
- Falls back to updating the window title bar if the viewport text fails
- Calls `get_display_filter_joints()` which reads from the shared `_display_filter_joints` list
- That list is updated by the file-watcher thread watching `opensim_joint_display_config.json`

**The joint-display watcher thread** (`_joint_display_watcher_thread`):
Polls `opensim_joint_display_config.json` every 50 ms. When the file's modification timestamp changes, it re-reads the JSON, validates the joint list, and (if the sequence number is newer than the last seen) updates `_display_filter_joints`. This is how a trigger from Open Ephys changes what the HUD shows — the plugin writes the file, this thread sees it within 50 ms.

**Session management:**
A "session" resets when: source changes, packet timestamp resets, or a gap > 500 ms between packets. On session reset, AHRS filters are recreated (to clear drift), calibration restarts, and frame history clears.

**Static detection:**
If gyro norm < 0.5 °/s and accel values barely change over time, the code flags the stream as "static". Static + live source → freeze the model instead of jittering. This prevents the skeleton from twitching when the operator is holding perfectly still.

---

### 10. `ephys_to_opensim_bridge.py` — The Batch Mode Bridge

**What it does in one sentence:** Collects a burst of IMU data, runs offline AHRS and then OpenSim batch IK on it, and opens the OpenSim GUI to show the replay.

**Simple analogy:** Instead of the live projector, this is the VCR — it records a clip, processes it, and plays it back.

**Modes:**
- `--seconds N` — collect for N seconds then process
- `--until-idle N` — collect until no packets for N seconds, then process
- `--max-samples N` — stop at N samples regardless

**Pipeline:**
1. Bind UDP port 5000
2. Receive packets (`_recv_packet`) — handles both raw accel/gyro and v2 quaternion format
3. Store all samples in memory
4. Run `imufusion.Ahrs` on each sensor's accel/gyro list → list of quaternions
5. Write a `.sto` orientation file (`samples_to_sto`)
6. Write an IK setup XML (`ensure_ephys_xml`)
7. Run `opensim-cmd.exe run-tool ephys_imuIK_Setup.xml`
8. Launch `OpenSim64.exe` showing the model + motion file

**Tibia-only mode:**
If only one sensor (the right shin/tibia sensor) is connected, the script locks every joint in the model *except* the right knee. This gives a clean single-joint angle without the solver going haywire on all the unconstrained joints.

---

### 11. `opensim_sensor_map.json` — The Sensor Slot Map

Written by the Open Ephys plugin just before launching the Python bridge. Tells the Python scripts which sensor slot index corresponds to which body segment name.

Example:
```json
{ "sensor_slots": ["tibia_r_imu", "femur_r_imu", "pelvis_imu", "torso_imu"] }
```

---

### 12. `opensim_display_joint.json` — The Single-Joint HUD Config

Written by the plugin when the operator changes the "Display Joint" dropdown. Tells the Python bridge which single joint to show in the simpler single-joint HUD and to send back over UDP to port 5001.

Example:
```json
{ "joint": "knee_angle_r", "label": "Right Knee", "joint_index": 1 }
```

---

### 13. `opensim_joint_display_config.json` — The Multi-Joint Trigger Config

Written by the plugin when a trigger fires (or when the joint selection changes). The Python bridge's watcher thread polls this file every 50 ms. When it changes, the HUD updates to show only the listed joints.

Example:
```json
{ "joints": ["knee_angle_r", "hip_flexion_r"], "trigger_ts": 1749123456.7, "seq": 3 }
```

The `seq` (sequence number) ensures old file writes are never applied after newer ones, even if the file system delivers them out of order.

---

### 14. `tests/vqf_decimation_ts_test.c` — VQF Timing Test

**What it does in one sentence:** Verifies that the sensor fusion module correctly uses each sensor's *own* sample rate (not the module's global rate) when initializing VQF.

**Why this matters:** If a sensor runs at 100 Hz but the module runs at 1000 Hz, VQF must be told `Ts = 1/100 = 0.01 s` — not `Ts = 1/1000 = 0.001 s`. Otherwise the gyroscope integration is 10× too fast and the quaternion spins 10× faster than reality.

---

### 15. `tests/redpitaya_stream_framing_test.c` — Binary Framing Test

**What it does in one sentence:** Builds a fake binary stream (including a garbage byte at the start to test resync), parses it with the same header-parsing logic used in `AcqBoardRedPitaya::run()`, and verifies it extracts both frames correctly.

**What it tests:**
1. The resync loop correctly skips the garbage byte and finds the first valid 22-byte header
2. Variable payload sizes work (e.g. firmware adds fusion channels mid-stream)
3. `samplesPerBuffer` is always ≥ 1 at all supported sample rates

---

## Data Flow Summary

```
Human moves body
  ↓ (mechanical motion)
IMU chips on stickers
  (accelerometer: measures linear acceleration)
  (gyroscope: measures angular velocity)
  (magnetometer: measures magnetic field direction)
  ↓ (raw int16 values over I2C or SPI bus)
RedPitaya_justin.c  ←────────────────┐
  ├─ sensor_fusion.c                 │  commands from PC
  │    └─ vqf.c           (CFG/FREQ/START/STOP)
  │         ↓ (quaternion Q15)
  ├─ pack into 22-byte header + payload
  └─ send over TCP ──────────────────────────────────────────────┐
                                                                  │
                                                                  ↓
AcqBoardRedPitaya::run() (Windows, C++/JUCE)              (TCP socket)
  ├─ parse header + payload
  ├─ scale int16 → float (using sensor presets)
  ├─ push to Open Ephys recording engine
  └─ send UDP broadcast on port 5000 ──────────────────────────────┐
                                                                    │
                     ┌──────────────────────────────────────────────┘
                     ↓ (UDP port 5000)
opensim_live_realtime.py (Python 3.8)
  ├─ _udp_ahrs_thread:
  │    ├─ parse v3/v2/legacy packet
  │    ├─ run imufusion AHRS (if v3/legacy)
  │    ├─ apply accel/gyro baseline calibration
  │    └─ push (t, quats, sensors) to frame_queue
  │
  ├─ _joint_display_watcher_thread:
  │    └─ poll opensim_joint_display_config.json every 50 ms
  │         └─ update _display_filter_joints on change
  │
  └─ run_live (main thread):
       ├─ pop latest frame from queue
       ├─ build OrientationsReference from quaternions
       ├─ ikSolver.assemble(state)  ← joint angles computed here
       ├─ _render_joint_display_hud()
       │    └─ reads _display_filter_joints
       │    └─ calls _read_coord_value() for each selected joint
       │    └─ updates DecorativeText or window title
       └─ model.getVisualizer().show(state)

  ↓ (UDP port 5001, angle feedback)
Open Ephys plugin (optional feedback loop)
```

---

## Key Numbers to Know

| Constant | Value | Meaning |
|---------|-------|---------|
| `UDP_PORT` | 5000 | Port Open Ephys sends IMU data on |
| `ANGLE_FEEDBACK_PORT` | 5001 | Port Python sends angle data back on |
| `SAMPLE_RATE` | 1000 Hz (firmware) / 100 Hz (bridge) | How many sensor readings per second |
| `LIVE_VISUALIZER_RATE` | 20 Hz | How fast OpenSim re-renders (env: `OPENSIM_LIVE_VISUALIZER_RATE`) |
| `CALIB_DURATION_S` | 3 s | How long the still-standing calibration takes |
| `FUSION_QUAT_Q15_SCALE` | 32767 | The multiplier for Q15 fixed-point encoding |
| `MAX_DISPLAY_JOINTS` | 6 | Max joints shown in the HUD at once |
| `HEADER_SIZE` | 22 bytes | Size of each binary frame header |

---

## Common Questions

**Q: Why quaternions instead of Euler angles (pitch/roll/yaw)?**
Euler angles have a famous problem called "gimbal lock" — when two rotation axes line up, you lose a degree of freedom and the math breaks down. Quaternions are immune to this. They are harder to visualize but they are what all serious orientation math uses internally.

**Q: Why Python 3.8 specifically?**
The OpenSim 4.5 SDK provides Python bindings compiled for Python 3.8. A different Python version will fail to import the `opensim` module.

**Q: What is the OpenSim model file (`.osim`)?**
It is an XML file describing a human musculoskeletal model: bones, joints, muscles, and the locations of the IMU attachment frames. The `Rajagopal2015_opensense_calibrated.osim` file is a standard full-body model calibrated with the sensor positions for this setup.

**Q: What is the IK constraint value `20.0`?**
The InverseKinematicsSolver constructor takes a "constraint weight" that balances matching the sensor orientations (weight: 20.0) vs. satisfying joint range-of-motion limits. Larger value = more trust in the sensor data.

**Q: Why is there a separate `opensim_joint_display_config.json` instead of a UDP command?**
Writing a JSON file and watching for changes is simpler and more reliable than adding a second UDP channel. The file is written atomically and the 50 ms polling delay is imperceptible for a trigger event.

---

## Appendix — Vocabulary and Concepts Reference

This appendix explains every technical term that appears in the codebase but is not fully defined elsewhere in this document.

---

### Motion and Orientation

**IMU (Inertial Measurement Unit)**
A tiny chip that measures three things: linear acceleration (accelerometer), rotation speed (gyroscope), and optionally magnetic field direction (magnetometer). Modern smartphones, game controllers, and fitness trackers all contain one.

**Accelerometer**
Measures the force of acceleration in three axes (X, Y, Z) in units of m/s² or g (where 1 g ≈ 9.807 m/s²). When stationary, it measures gravity. Because gravity always pulls downward, a still accelerometer tells you which way is "down."

**Gyroscope**
Measures angular velocity — how fast the sensor is spinning around each axis — in degrees/second or radians/second. Integrating (summing) angular velocity over time gives rotation angle, but tiny errors add up (this is called gyro drift).

**Magnetometer**
Measures the strength and direction of magnetic fields in microtesla (µT). Used like a compass to detect Earth's magnetic north, which corrects the left-right spin (yaw) that a gyroscope alone cannot reliably track.

**Sensor Fusion**
Combining readings from multiple sensors to produce an estimate better than any single sensor alone. VQF fuses gyro (fast but drifts), accelerometer (slow but stable for tilt), and magnetometer (gives absolute heading).

**Quaternion**
A mathematical object using four numbers (w, x, y, z) to represent a 3D rotation without the gimbal-lock problem of Euler angles. A unit quaternion has |w² + x² + y² + z²| = 1. Identity (no rotation) is [1, 0, 0, 0].

**Euler Angles (Pitch, Roll, Yaw)**
The familiar "tilt forward/backward, tilt side-to-side, turn left/right" angles. Intuitive but mathematically problematic: when pitch reaches ±90°, roll and yaw become undefined (gimbal lock). VQF uses quaternions internally and converts to degrees only at the output stage for display.

**Gimbal Lock**
When two rotation axes line up, you lose a degree of freedom and can no longer express all possible orientations with Euler angles. Quaternions are immune to this because they represent rotation differently.

**AHRS (Attitude and Heading Reference System)**
A system that tracks a device's full 3D orientation (attitude = pitch/roll, heading = yaw) in real-time using an IMU. VQF and `imufusion.Ahrs` are both AHRS algorithms.

**Strapdown Integration**
The process of integrating gyroscope readings over time to track orientation without a physical gyroscope gimbal. The quaternion update `q_new = q_old × ΔQ` where `ΔQ` encodes the gyro reading is strapdown integration.

**Gyro Bias**
Every gyroscope chip reports a small non-zero rotation rate even when perfectly still. This offset (bias) must be estimated and subtracted. VQF estimates it continuously; the Python bridge does a 3-second still calibration for the `imufusion` library.

**Rest Detection**
VQF monitors whether the sensor has been still long enough (default 1.5 s) for highly accurate gyro bias estimation. The `FUSION_STATUS_REST_DETECTED` flag signals this state.

**Magnetic Disturbance**
When a metal object, electric motor, or power cable distorts Earth's magnetic field near the sensor. VQF tracks the expected field norm and dip angle; if the current reading diverges too far it sets `FUSION_STATUS_MAG_DISTURBANCE` and reduces or disables the magnetometer correction until the disturbance clears.

**Dip Angle**
The angle between Earth's magnetic field vector and the horizontal plane. Varies by location (around 60–70° in most of North America). VQF uses it as a fingerprint to confirm the magnetometer is seeing real Earth field, not a local disturbance.

---

### Signal Processing

**Low-Pass Filter (LPF)**
A filter that passes slow changes and smooths out fast noise. VQF uses a 2nd-order Butterworth LPF on the accelerometer to average out real motion accelerations before using the measurement for tilt correction.

**Butterworth Filter**
A type of low-pass filter with a maximally flat frequency response (no ripple in the passband). VQF uses 2nd-order Butterworth filters for the accelerometer, rest detection, and magnetometer norm tracking.

**Time Constant (τ, tau)**
A measure of how fast a filter responds. A first-order LPF with time constant τ reaches 63% of a step change after τ seconds. In VQF, `tauAcc = 3.0` means the accel tilt correction takes about 3 seconds to fully correct a sudden tilt error.

**Q15 Fixed-Point Format**
A way to store floating-point numbers in integers to save memory and bandwidth. Q15 means there are 15 bits after the (implied) binary point, so the range [-1, +1] maps to integers [-32768, +32767]. Converting: `int16 = float × 32767` (encode); `float = int16 / 32767.0` (decode).

**Dead Reckoning**
Navigation by integrating known velocity/angular-velocity over time from a starting position/orientation. Pure gyro strapdown integration is dead reckoning — accurate for short times but accumulates error.

**Decimation**
Running a sensor at a lower effective rate than the main clock. If the hardware clock ticks at 1000 Hz but the IMU produces readings at 100 Hz, the sensor is decimated 10:1. VQF must know the actual per-sensor rate to integrate correctly.

---

### Inverse Kinematics and OpenSim

**IK (Inverse Kinematics)**
Given the orientation of each body segment (from the IMU sensors), calculate the joint angles (knee flexion, hip flexion, etc.) that would produce those orientations. The "inverse" part means you start with the result (orientations) and work backward to find the causes (joint angles). This is computationally harder than "forward kinematics" (given joint angles, find segment orientations).

**OpenSim**
An open-source biomechanics simulation software by Stanford/NMSM. It contains a library of musculoskeletal models (bones, joints, muscles) and tools like the InverseKinematicsSolver.

**Simbody**
The physics engine inside OpenSim. The 3D skeleton viewer window is Simbody's built-in visualizer.

**InverseKinematicsSolver**
The OpenSim class that solves IK. It takes an OrientationsReference (the sensor data) and minimizes the total error between measured orientations and the model's predicted orientations by adjusting joint coordinates.

**OrientationsReference**
An OpenSim object that wraps a table of time-stamped quaternion orientations (one per sensor frame). Passed to the IK solver as the target to match.

**CoordinateSet**
The collection of all joint degrees of freedom in an OpenSim model. Each Coordinate has a name (like `knee_angle_r`), a value (in radians), and optional range limits.

**TimeSeriesTableRotation**
An OpenSim data table where each column is a rotation matrix (converted from a quaternion) for one sensor. The IK solver reads from this table.

**`.osim` file**
XML file describing an OpenSim musculoskeletal model. Defines bones (Bodies), joints (with Coordinates), muscles, and attachment points for IMU sensors (PhysicalOffsetFrames).

**`.sto` file**
OpenSim Storage file — a tab-separated text file with a header section followed by time-series data. Used to store quaternion orientations (input to IK) and solved joint angle trajectories (output of IK).

**PhysicalOffsetFrame**
In the OpenSim model, the virtual "attachment point" where an IMU sensor is placed on a body segment. The frame name (like `tibia_r_imu`) must exactly match the column labels in the orientation data for the IK solver to pair them correctly.

**Constraint Weight**
A parameter in the IK solver controlling how strongly it tries to match sensor orientations vs. respecting joint range-of-motion limits. Value 20.0 means orientation matching is weighted 20× more than range-of-motion violation.

**Rajagopal2015 Model**
The standard full-body musculoskeletal model from the paper by Rajagopal et al. (2015). Contains 37 joint coordinates and pre-defined IMU attachment frame locations, making it the standard choice for full-body OpenSense motion capture.

---

### Networking and Data Transport

**UDP (User Datagram Protocol)**
A connectionless network protocol where packets are sent independently with no delivery guarantee. Chosen here because it is fast and low-latency — a dropped packet just means one frame is missed, which is acceptable for real-time motion capture.

**TCP (Transmission Control Protocol)**
A connection-based network protocol with guaranteed delivery and ordering. Used for the Red Pitaya → Open Ephys data stream because every sample must arrive and in order.

**Socket**
A programming interface to a network connection. Think of it as a two-way postal mailbox with an address (IP:port).

**Port**
A number (0–65535) that distinguishes different services on the same IP address. This project uses: 5000 for IMU data (Red Pitaya → Open Ephys, then Open Ephys → Python), 5001 for angle feedback (Python → Open Ephys), and internal TCP port for Red Pitaya streaming.

**Broadcast**
Sending one UDP packet to all devices on the local network simultaneously. Open Ephys uses broadcast to distribute trigger signals to all connected processes.

**Latency**
The delay from when something happens to when software detects it. The 50 ms poll interval for the JSON file watcher means a trigger takes at most 50 ms + one IK frame time to appear on the HUD.

---

### Software Architecture

**JUCE**
A C++ framework for building audio/signal-processing applications with graphical interfaces, commonly used in neuroscience recording software. Open Ephys is built on JUCE.

**Plugin / DataThread**
In Open Ephys, a plugin is a shared library (.dll/.so) that adds new hardware support. The Red Pitaya plugin implements the DataThread interface, which Open Ephys calls for each hardware tick.

**Abstract Class / Pure Virtual**
A C++ class where some methods are declared but not implemented (they have `= 0` in the declaration). It is a contract: any subclass must implement those methods. `AcquisitionBoard` (Acqboard.h) is an abstract class.

**Thread Safety / Mutex Lock**
When two threads share data, a "race condition" can corrupt the data if both write simultaneously. A `mutex` (mutual exclusion lock) prevents this: only one thread can hold the lock at a time. In Python, `threading.Lock()` provides this. In the code, `_frame_lock`, `_display_filter_lock`, and `_ag_calib_lock` each protect one piece of shared state.

**Daemon Thread**
A thread that automatically stops when the main program exits, without needing an explicit stop signal. Both background threads in `opensim_live_realtime.py` (`_udp_ahrs_thread` and `_joint_display_watcher_thread`) are daemon threads.

**Atomic Write**
Writing a file in a way that guarantees readers either see the old file or the new complete file — never a half-written file. Typically done by writing to a temporary file first, then renaming (rename is atomic on most file systems). Used when writing the JSON sidecar files.

**Frame / Sample**
In this context, one "frame" is one time slice of all sensor data (one reading from every active IMU). A "sample" sometimes means the same thing, and sometimes means one value from one channel.

**Binary Stream**
Data sent over a network as raw bytes (not text). The Red Pitaya uses a binary stream with a fixed 22-byte header + variable payload for efficiency.

---

### Firmware / Embedded

**Red Pitaya**
A credit-card-sized Linux computer with an FPGA coprocessor. The FPGA handles real-time hardware timing; the ARM CPU runs the C firmware and the network stack.

**FPGA (Field-Programmable Gate Array)**
A chip whose digital logic can be configured after manufacture. The Red Pitaya's FPGA handles precise hardware timing and the GPIO/I2C bus interface.

**AXI (Advanced eXtensible Interface)**
A standard interface for connecting peripherals to the ARM processor in an FPGA SoC. On Red Pitaya, the FPGA peripherals (GPIO counter, I2C controller) are accessed by mapping their AXI registers into ARM memory space.

**I2C (Inter-Integrated Circuit)**
A two-wire serial communication bus (SDA data, SCL clock) for slow peripheral chips. All four of the Red Pitaya's IMU slots on the I2C bus use this protocol. The IMU registers are read by the firmware over I2C.

**SPI (Serial Peripheral Interface)**
A faster four-wire serial bus. Two Red Pitaya IMU slots support SPI for higher-speed sensors or when I2C is too slow.

**GPIO (General Purpose Input/Output)**
Digital pins that can be configured as inputs or outputs. On the Red Pitaya, a hardware counter register is connected to GPIO and used to time the sample loop precisely.

**`mmap` (Memory-Mapped I/O)**
Technique for accessing hardware registers as if they were ordinary memory addresses. `mmap(AXI_GPIO_ADDRESS, RANGE, ...)` makes the FPGA's GPIO counter appear as a pointer the C firmware can read directly.

**Watchdog**
A mechanism that detects when a program has hung and restarts it. RedPitaya_justin.c uses `setjmp`/`longjmp` with a signal handler as a lightweight watchdog for the streaming loop.

**`volatile`**
A C keyword that tells the compiler "this variable can be changed by hardware or another thread at any time — do not cache it in a register." Used for `g_stream_hw_hz` (updated by command parsing) and `stop_requested` (updated by signal handler).

---

### Configuration and File Formats

**JSON (JavaScript Object Notation)**
A human-readable text format for storing structured data as key-value pairs, arrays, and nested objects. Used for all runtime configuration files in this project.

**`.json` sidecar file**
A small JSON file written alongside the main data files to pass configuration from one process to another. This project uses three: `opensim_sensor_map.json`, `opensim_display_joint.json`, and `opensim_joint_display_config.json`.

**Sequence Number (`seq`)**
An integer that increases by 1 each time a config is written. The file-watcher only applies an update when `seq > last_seen_seq`, which prevents a slow write from overwriting a later, faster write.

**Environment Variable**
A named string value set in the operating system process environment. Programs read them via `os.environ.get("NAME", default)`. Used here for: `OPENSIM_LIVE_VISUALIZER_RATE`, `OPENSIM_CALIB_DURATION`, `OPENSIM_SKIP_CALIB`, `OPENSIM_LIVE_SOURCE`, `ESP32_NODE_HOST`.

**`mtime` (modification time)**
A timestamp stored by the file system recording when a file was last written. Used by `_joint_display_watcher_thread` to detect file changes without reading the entire file on every poll.

