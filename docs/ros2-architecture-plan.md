# ROS2 Architecture Plan

## Overview

This document covers the decision to adopt ROS2 as the communication middleware
for the next-generation system integrating Jetson, Red Pitaya, EMG, IMU,
motor control, and VR under a unified Rust-based GUI.

### References

- ROS2 Rolling Documentation: https://docs.ros.org/en/rolling/index.html
- SeedStudio ROS2 Humble on Jetson: https://wiki.seeedstudio.com/install_ros2_humble/
- Klein et al. (2025), "A real-time full-chain wearable sensor-based musculoskeletal simulation: an OpenSim-ROS Integration" — arXiv:2507.20049

---

## Summary

We are moving from the current Open Ephys C++/JUCE plugin to a standalone
system built on ROS2 Humble running on a Jetson (SeedStudio reComputer). Each
device — Red Pitaya (IMU), EMG, motor controller, VR headset — becomes its own
ROS2 node. Nodes talk through DDS publish/subscribe instead of our hand-written
TCP/UDP protocols. The Red Pitaya firmware stays unchanged; a bridge node on the
Jetson reads its existing TCP frames and republishes them as standard ROS2
`sensor_msgs/Imu` messages. OpenSim IK runs as a ROS2 node following the
approach validated by Klein et al. (2025), who measured ~18 ms ROS2 transport
overhead and 99.1% frame delivery within 500 ms on a full IK+ID+SO pipeline.
The GUI will be a LabVIEW-style web interface (Tauri or rosbridge) subscribing
to ROS2 topics for display and publishing commands back. This gives us one
communication layer across all devices, automatic data recording via `ros2 bag`,
and the ability to add new sensors or processing nodes without touching existing
ones.

---

## Why ROS2

The current system uses hand-written TCP/UDP protocols between the Open Ephys
GUI plugin (C++/JUCE) and the Red Pitaya firmware. Each communication link
has its own binary frame format, byte packing, and thread management. Adding
new devices (Jetson motors, EMG, VR) would require writing and maintaining
separate protocol implementations for each.

ROS2 replaces all of this with a single publish/subscribe middleware built on
DDS (Data Distribution Service). Nodes publish typed messages to named topics;
subscribers receive them. The transport layer (TCP, UDP, shared memory),
serialization, buffering, and device discovery are all handled by DDS.

### What ROS2 Replaces

| Current system (manual)          | ROS2 equivalent                        |
|----------------------------------|----------------------------------------|
| TCP socket creation/management   | DDS transport (automatic)              |
| 22-byte binary frame headers     | Typed message definitions              |
| Q15 fixed-point encoding         | Native float fields in messages         |
| ASCII command protocol           | ROS2 services (request/response)       |
| UDP packet packing/unpacking     | Publisher/subscriber with QoS          |
| Frame resync logic               | Not needed (DDS guarantees framing)    |
| TCP_NODELAY (Nagle's algorithm)  | QoS profiles (best-effort/reliable)    |
| Ring buffers between threads     | DDS internal queuing                   |
| Thread synchronization           | Nodes are separate processes            |
| Device discovery                 | DDS auto-discovery                     |

---

## ROS2 Core Concepts

### Node
A single process that does one thing. Examples: "read IMU data," "solve IK,"
"control motor." Each node is its own executable.

### Topic
A named channel that nodes publish and subscribe to. Examples:
`/imu/data`, `/joint_states`, `/motor/command`. Publishers and subscribers are
decoupled — publishers don't know who is listening.

### Message
A typed, structured data format that travels on a topic. Defined in `.msg`
files with versioned fields. ROS2 provides standard message types for common
sensor data:

```
# sensor_msgs/msg/Imu.msg
Header header                          # timestamp + frame_id
geometry_msgs/Quaternion orientation    # qw, qx, qy, qz
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
```

### Service
Request/response pattern for one-off commands. Example: "calibrate sensor"
returns "calibration complete."

### Action
Like a service but for long-running tasks with progress feedback. Example:
"move motor to 90 degrees" with incremental position updates.

### DDS (Data Distribution Service)
The underlying transport protocol. Handles TCP, UDP, shared memory,
serialization, discovery, and delivery guarantees. Application code never
touches sockets directly.

### QoS (Quality of Service)
Per-topic reliability settings:

| QoS Policy   | Behavior                          | Use case               |
|--------------|-----------------------------------|------------------------|
| Reliable     | Guaranteed delivery (TCP-like)    | Commands, config       |
| Best-effort  | Drop if slow (UDP-like)           | Sensor streams, video  |

---

## Evidence: Klein et al. (2025) — OpenSim-ROS Integration

### What They Built

A real-time full-chain wearable sensor-based musculoskeletal simulation
integrating OpenSimRT with ROS. The pipeline:

```
IMU sensors (XIMU-3, 100 Hz, UDP over WiFi)
    |
    v
[IK Node] — Inverse Kinematics using OpenSimRT
    |         Uses ROS TF (transform library) for sensor-to-body mapping
    |         Quaternion calibration via SVD-based algorithm
    v
[ID Node] — Inverse Dynamics (Newton-Euler)
    |         Receives joint angles (q, q_dot, q_ddot) + external forces
    |         Pressure insole data synchronized via buffer + time correction
    v
[SO Node] — Static Optimization (multithreaded)
              Minimizes sum of squared muscle activations
              Deterministic pipeline scheduler, 4-12 threads
```

### Key Architecture Decisions

- OpenSimRT functionalities wrapped as ROS nodes inside Docker containers
- OpenSim `.osim` model converted to URDF using Pinocchio for ROS compatibility
- Symbolic moment-arm library generated from scaled OpenSim model
- ROS TF library used as generic orientation wrapper (supports both IMU and AR markers)
- Joint mapping defined between OpenSim and URDF coordinate frames

### Measured Latency (Table III from paper)

| Pipeline stage         | Event pair | 4 threads | 6 threads | 12 threads |
|------------------------|------------|-----------|-----------|------------|
| IK-to-ID sync          | 0-1        | 260.60 ms | 260.63 ms | 260.69 ms  |
| ID computation         | 5-6        | 0.11 ms   | 0.10 ms   | 0.10 ms    |
| ID-to-SO transport     | 6-7        | 23.70 ms  | 19.25 ms  | 18.28 ms   |
| SO queue wait          | 7-8        | 55.89 ms  | 21.86 ms  | 2.91 ms    |
| SO computation         | 8-9        | 22.93 ms  | 26.58 ms  | 28.41 ms   |
| **Full pipeline mean** | All        | 363.26 ms | 328.44 ms | 310.42 ms  |
| **95th percentile**    |            | 590 ms    | 480 ms    | 401 ms     |

Notes:
- The 260 ms IK-to-ID delay was caused by pressure insole buffering, not ROS2 overhead
- Actual ROS2 message-passing overhead (events 6-7) was only 18-24 ms
- With 12 threads and a 500 ms deadline, 99.1% of frames were delivered on time
- Events 1-5 took less than 0.02 ms each (omitted from table)
- System tested on Intel Core i7-10750H running Debian GNU/Linux 11

### IK Accuracy (Table II from paper)

| Activity | Joint | RMSE (degrees) |
|----------|-------|----------------|
| Walking  | Hip   | 4.6 - 7.7      |
| Walking  | Knee  | 3.9 - 4.0      |
| Walking  | Ankle | 5.1 - 6.2      |
| Squat    | Hip   | 9.5 - 13.0     |
| Squat    | Knee  | 13.7 - 14.1    |
| Squat    | Ankle | 6.1 - 9.1      |

Compared against offline Mocap-based OpenSim analysis.

### Relevance to Our Project

This paper validates that:
1. ROS2 middleware does not introduce prohibitive latency for real-time biomechanics
2. OpenSimRT can be wrapped as ROS nodes without architectural issues
3. IMU-to-IK-to-visualization pipeline works at human-movement timescales
4. Docker containerization of the full stack is viable for reproducibility

The 260 ms insole sync delay does not apply to our system (we use direct IMU streaming
without pressure insoles), so our expected pipeline latency is closer to 50-100 ms.

---

## SeedStudio / Jetson ROS2 Setup

### Hardware
SeedStudio reComputer with NVIDIA Jetson. Supported JetPack versions: 5.1.2 and 6.2.

### Installation (ROS2 Humble)

```bash
# 1. Locale
locale  # check for UTF-8
sudo update-locale LANG=en_US.UTF-8

# 2. Dependencies
sudo apt install gnupg wget software-properties-common -y

# 3. Add NVIDIA Isaac ROS + ROS2 repositories
# (US or China mirrors available — see SeedStudio wiki)

# 4. Install ROS2 Humble Desktop
sudo apt install ros-humble-desktop -y

# 5. Build tools
sudo apt install ros-dev-tools -y

# 6. Initialize rosdep
sudo rosdep init
rosdep update

# 7. Source environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 8. Verify
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

### Implications for Our System
- Jetson runs ROS2 natively — all nodes can execute on the Jetson directly
- Red Pitaya still communicates via its existing TCP protocol; a bridge node
  on the Jetson converts TCP frames into ROS2 topics
- Motor control node runs directly on Jetson with GPIO/serial access
- GPU acceleration available for any compute-heavy nodes (IK, filtering)

---

## Proposed System Architecture

### Node Graph

```
┌─ Jetson (ROS2 Humble) ──────────────────────────────────────────────┐
│                                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │
│  │ rp_imu_node  │  │ emg_node     │  │ motor_node   │               │
│  │ (C++ or Py)  │  │ (C++ or Py)  │  │ (C++ or Rust)│               │
│  │              │  │              │  │              │               │
│  │ Reads Red    │  │ EMG acquire  │  │ Jetson GPIO/ │               │
│  │ Pitaya TCP,  │  │ + envelope   │  │ serial to    │               │
│  │ converts to  │  │ extraction   │  │ motor driver │               │
│  │ ROS Imu msgs │  │              │  │              │               │
│  └──────┬───────┘  └──────┬───────┘  └──────▲───────┘               │
│         │ publish         │ publish         │ subscribe              │
│         ▼                 ▼                 │                        │
│    /imu/data         /emg/raw          /motor/command                │
│    /imu/quaternions  /emg/envelope     /motor/state                  │
│         │                 │                 ▲                        │
│  ┌──────┴─────────────────┴─────────────────┴────────────────────┐  │
│  │                    ROS2 DDS Middleware                         │  │
│  └──────┬─────────────────┬─────────────────┬────────────────────┘  │
│         │                 │                 │                        │
│         ▼                 ▼                 │                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────┴───────┐               │
│  │ ik_node      │  │ vr_node      │  │ gui_node     │               │
│  │ (C++)        │  │ (C++)        │  │ (Rust/Tauri  │               │
│  │              │  │              │  │  or rosbridge│               │
│  │ OpenSimRT    │  │ OpenXR       │  │  + web UI)   │               │
│  │ IK solver    │  │ headset/     │  │              │               │
│  │              │  │ controllers  │  │ LabVIEW-style│               │
│  └──────┬───────┘  └──────────────┘  │ control panel│               │
│         │ publish                     └──────────────┘               │
│         ▼                                                            │
│    /joint_states                                                     │
│    (sensor_msgs/JointState)                                          │
└──────────────────────────────────────────────────────────────────────┘
```

### Node Specifications

| Node           | Language    | Subscribes to                    | Publishes to                  | Function                                    |
|----------------|-------------|----------------------------------|-------------------------------|---------------------------------------------|
| `rp_imu_node`  | C++ or Py   | (Red Pitaya TCP hardware)        | `/imu/data`, `/imu/quaternions` | Bridge: RP TCP frames to ROS Imu messages |
| `emg_node`     | C++ or Py   | (EMG hardware)                   | `/emg/raw`, `/emg/envelope`   | EMG acquisition + envelope extraction       |
| `motor_node`   | C++ or Rust | `/motor/command`                 | `/motor/state`                | Jetson GPIO/serial to motor driver          |
| `ik_node`      | C++         | `/imu/data`                      | `/joint_states`               | OpenSimRT inverse kinematics                |
| `id_node`      | C++         | `/joint_states`                  | `/joint_torques`              | Inverse dynamics (if needed)                |
| `vr_node`      | C++         | `/joint_states`, `/emg/envelope` | `/vr/events`                  | OpenXR visualization                        |
| `gui_node`     | Rust        | all of the above                 | `/trigger`, `/motor/command`  | LabVIEW-style control panel                 |
| `recorder_node`| Python      | all topics                       | (rosbag files)                | `ros2 bag record` for experiment data       |

### Standard ROS2 Message Types Used

```
sensor_msgs/msg/Imu              — IMU data (quaternion, angular vel, linear accel)
sensor_msgs/msg/JointState       — joint names, positions, velocities, efforts
geometry_msgs/msg/WrenchStamped  — forces and torques (for ID)
std_msgs/msg/Float64MultiArray   — EMG envelope, muscle activations
trajectory_msgs/msg/JointTrajectory — motor commands (position/velocity targets)
```

### QoS Configuration

| Topic             | QoS Policy   | Rationale                                       |
|-------------------|--------------|------------------------------------------------ |
| `/imu/data`       | Best-effort  | High rate, dropped frame = stale render, not corruption |
| `/emg/raw`        | Best-effort  | Same as IMU — continuous stream                 |
| `/joint_states`   | Reliable     | IK output feeds ID and visualization            |
| `/motor/command`  | Reliable     | Motor commands must not be dropped               |
| `/trigger`        | Reliable     | Trigger events must be delivered to all subscribers |
| `/motor/state`    | Best-effort  | Feedback for display, not control-critical       |

---

## GUI Architecture Options

### Option A: ROS2 Everywhere
Every component is a ROS2 node including the GUI. GUI uses `rclrs` (Rust ROS2 bindings).

- One communication system
- `ros2 bag` records everything automatically
- `rviz2` provides free 3D visualization
- Risk: `rclrs` bindings are immature

### Option B: Hybrid (ROS2 for data, Rust for control)
Sensor nodes and IK in ROS2. GUI and motor control are native Rust, bridging
into ROS2 only for data exchange.

- Tight motor control loop stays in native Rust (lowest latency)
- GUI doesn't depend on `rclrs` maturity
- Risk: two communication systems to maintain

### Option C: ROS2 Backbone + Tauri GUI (Recommended)
All device nodes in ROS2 (C++/Python). GUI is a Tauri app (Rust backend + web
frontend) connected to ROS2 via rosbridge (WebSocket/JSON).

- Mature ROS2 nodes in C++/Python
- LabVIEW-style web GUI achievable with existing web component libraries
- rosbridge is well-tested for web-based robot GUIs
- Tradeoff: WebSocket adds ~5-10 ms latency to GUI updates (acceptable for display)

---

## Transition from Current System

### What Stays
- Red Pitaya firmware and its TCP protocol (hardware doesn't change)
- OpenSim musculoskeletal models (.osim files)
- Python OpenSim bindings for IK (wrapped as a ROS node instead of subprocess)
- Sensor mapping configuration (opensim_sensor_map.json)
- UDP transport to OpenSim if retained for backward compatibility

### What Changes
| Current                        | New                                    |
|--------------------------------|----------------------------------------|
| JUCE StreamingSocket           | ROS2 node reads TCP, publishes to topic |
| JUCE DatagramSocket            | ROS2 publisher (DDS transport)         |
| JUCE DataBuffer (60k ring)     | DDS internal queuing                   |
| JUCE Thread subclass           | Separate ROS2 node processes           |
| Open Ephys broadcast trigger   | ROS2 `/trigger` topic (reliable QoS)   |
| Python subprocess for OpenSim  | OpenSimRT ROS node (per Klein et al.)  |
| JSON config via JUCE File      | ROS2 parameter server + launch files   |
| Open Ephys plugin host         | Standalone Tauri application            |

### Wire Protocol Unchanged
The Red Pitaya firmware still sends the same 22-byte TCP frame headers with
Q15 int16 payloads. The `rp_imu_node` bridge speaks this protocol on the
device side and publishes standard `sensor_msgs/Imu` messages on the ROS2 side.
No firmware update required.

---

## ESP32-S3 Battery Optimizations

Three power-saving techniques for wireless sensor nodes (XIAO ESP32-S3):

1. **Drop CPU to 80 MHz** (`setCpuFrequencyMhz(80)`) — Cuts baseline draw from
   ~40 mA to ~20 mA. Still fast enough for IMU reads, sensor fusion, and ESP-NOW
   at 200 Hz. No functional downside.

2. **De-initialize radio between bursts** (`esp_now_deinit()`) — Collect N samples
   in a buffer, transmit as a batch, shut down the radio until the next batch.
   Saves significant power but adds latency equal to batch duration. A 10-sample
   batch at 200 Hz adds 50 ms, acceptable for body-segment IMUs doing IK. Not
   suitable for sensors feeding closed-loop motor control.

3. **Use unicast, not broadcast** — Transmit to specific MAC addresses instead of
   broadcasting. Faster transmission, lower power, and enables ESP-NOW delivery
   ACKs (broadcast has no ACK). No downside since sensor-to-hub mappings are fixed.

Power profile should be per-node configurable: aggressive batching for
kinematics-only IMUs, continuous streaming for any sensor in a low-latency
control loop.

---

## Key Risks and Mitigations

| Risk                                        | Mitigation                                              |
|---------------------------------------------|---------------------------------------------------------|
| `rclrs` (Rust ROS2 bindings) immaturity     | Use Option C: C++/Python nodes + rosbridge for GUI      |
| DDS latency for motor closed-loop control   | Motor node runs on Jetson with direct hardware access   |
| VR integration through ROS2 is awkward      | VR node uses OpenXR natively, subscribes to ROS2 topics |
| Clock synchronization across devices        | Start with host-side timestamps; upgrade to PTP if needed |
| OpenSimRT Docker container complexity       | Follow Klein et al. containerization pattern            |
