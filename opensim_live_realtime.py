"""
opensim_live_realtime.py  -  Python 3.8 ONLY
Real-time OpenSim IK with Simbody visualizer.
Receives UDP from Open Ephys on port 5000:
  - v3 accel/gyro (t, version=3, N, N×ax,ay,az,gx,gy,gz in g and deg/s) — preferred
  - v2 quaternion packets (t, version=2, N, N×qw,qx,qy,qz)
  - legacy acc/gyro IMU packets (no version tag)
Accel/gyro baseline calibration runs in this process: first 3 s still per session,
average a,g per sensor slot is subtracted before AHRS → IK.

=============================================================================
 HOW THIS SCRIPT WORKS — BIG PICTURE
=============================================================================
 Three things run concurrently (two daemon threads + main thread):

   Thread 1: _udp_ahrs_thread
     Listens on UDP port 5000 forever.
     For every packet received it:
       1. Parses the packet format (v3 / v2 / legacy).
       2. Detects session resets (timestamp jump, source change, long gap).
       3. Accumulates 3 s of still data for accel/gyro baseline calibration.
       4. Runs imufusion.Ahrs per sensor (v3/legacy only) to get quaternions.
       5. Applies the OpenSim frame rotation (_Q_OPENSIM_FRAME).
       6. Pushes the latest frame into _frame_queue (cleared each push so
          the main thread always gets the NEWEST frame, never stale ones).

   Thread 2: _joint_display_watcher_thread
     Polls opensim_joint_display_config.json every 50 ms.
     When the file's modification time changes it re-reads and validates the
     joint list, then updates _display_filter_joints under a lock.
     This is how the Open Ephys plugin triggers a HUD change without needing
     an extra network port: it writes the file; this thread notices within 50 ms.

   Main thread: run_live
     Loads the OpenSim model, sets up the IK solver, starts the other two
     threads, then loops at up to LIVE_VISUALIZER_RATE (default 20 Hz):
       1. Pop latest frame from _frame_queue.
       2. Build an OrientationsReference from the quaternions.
       3. Call ikSolver.assemble(state) — OpenSim solves for joint angles.
       4. Read selected joint angles from model.getCoordinateSet().
       5. Update the HUD (DecorativeText in the viewport or window title).
       6. Call model.getVisualizer().show(state) to render the skeleton.

=============================================================================
"""

import sys
import os
import socket
import struct
import time
import threading
import math
import select

if sys.version_info[:2] != (3, 8):
    sys.exit(f"ERROR: Must run with Python 3.8, got {sys.version}")

OPENSIM_PYTHON = r"C:\OpenSim 4.5\sdk\Python"
OPENSIM_BIN = r"C:\OpenSim 4.5\bin"
OPENSIM_SDK_LIB = r"C:\OpenSim 4.5\sdk\lib"

if OPENSIM_PYTHON not in sys.path:
    sys.path.insert(0, OPENSIM_PYTHON)

for _dll_dir in (OPENSIM_BIN, OPENSIM_SDK_LIB, os.path.join(OPENSIM_PYTHON, "opensim")):
    if os.path.isdir(_dll_dir):
        os.add_dll_directory(_dll_dir)

os.environ["PATH"] = OPENSIM_BIN + os.pathsep + os.environ.get("PATH", "")

import opensim as osim
import imufusion
import numpy as np

import json

import opensim_joint_catalog

# ── UDP packet version tags ───────────────────────────────────────────────────
# Packets start with: [timestamp_f32, version_f32, n_sensors_f32, ...data...]
# These constants identify which format the rest of the payload uses.
UDP_PACKET_VERSION_QUAT   = 2.0   # payload = N×{qw,qx,qy,qz}  (pre-fused quaternions)
UDP_PACKET_VERSION_IMU    = 3.0   # payload = N×{ax,ay,az,gx,gy,gz}  (raw accel+gyro)
UDP_PACKET_VERSION_ANGLES = 3.1   # outgoing angle feedback sent TO port 5001

# Angle feedback: after each IK solve, the selected joint angle is sent here
# so the Open Ephys plugin (or any listener) can log/display the raw value.
ANGLE_FEEDBACK_IP   = "127.0.0.1"
ANGLE_FEEDBACK_PORT = 5001

JOINT_OPTIONS = [
    ("hip_flexion_r", "Right Hip Flexion", 0),
    ("knee_angle_r", "Right Knee", 1),
    ("ankle_angle_r", "Right Ankle", 2),
    ("hip_flexion_l", "Left Hip Flexion", 3),
    ("knee_angle_l", "Left Knee", 4),
    ("ankle_angle_l", "Left Ankle", 5),
    ("pelvis_tilt", "Pelvis Tilt", 6),
    ("pelvis_list", "Pelvis List", 7),
    ("pelvis_rotation", "Pelvis Rotation", 8),
    ("lumbar_extension", "Lumbar Extension", 9),
]
JOINT_DISPLAY_CONFIG = "opensim_joint_display_config.json"
_sensor_map_cache = None
_display_joint_cache = None
_display_joint_mtime = None

try:
    osim.Logger.setLevelString("Warn")
except Exception:
    pass

MODEL_PATH = r"C:\Users\justi\Open-Sim--Bio-Mech\Rajagopal2015_opensense_calibrated.osim"
WORK_DIR = r"C:\Users\justi\Open-Sim--Bio-Mech"
UDP_IP = "0.0.0.0"
UDP_PORT = 5000
SAMPLE_RATE = 1000.0
LIVE_VISUALIZER_RATE = float(os.environ.get("OPENSIM_LIVE_VISUALIZER_RATE", "20.0"))
CONSTRAINT = 20.0
MAX_WAIT_S = 60.0
PACKET_LOG_INTERVAL = 100
SOURCE_LABEL = os.environ.get("OPENSIM_LIVE_SOURCE", "unknown").strip().lower()
LIVE_MODE         = True   # only real live IMU data drives the model
ALLOW_SAMPLE_DATA = False  # blocks flat/neutral fake packets when LIVE_MODE is True
NO_DATA_TIMEOUT_S = 5.0   # seconds of no packets before freezing model and showing error
SESSION_GAP_S  = 0.5
STALE_PACKET_S = 0.2
STATIC_NEUTRAL_S = 0.5
STATIC_GYRO_EPS = 0.5
STATIC_ACCEL_EPS = 0.03
STATIC_CHANGE_EPS = 0.02
STATIC_COORD_EPS_DEG = 0.25
REAL_SOURCE_LABELS   = ("real", "real_redpitaya", "redpitaya", "rp")
CALIB_DURATION_S = float(os.environ.get("OPENSIM_CALIB_DURATION", "3.0"))
OPENSIM_SKIP_CALIB = os.environ.get("OPENSIM_SKIP_CALIB", "0") == "1"

# ── Sensor slot layout ────────────────────────────────────────────────────────
# SENSORS is the canonical ordered list of IMU frame names used by the OpenSim
# model (Rajagopal2015_opensense_calibrated.osim).  Slot 0 is torso, slot 7 is
# left calcaneous.  All arrays that hold per-sensor data are indexed by these
# slot numbers.
#
# The IK solver requires that orientation labels in the TimeSeriesTableRotation
# exactly match frame names in the .osim model — use these exact strings.
SENSORS = [
    "torso_imu",     # slot 0 — trunk / thorax
    "pelvis_imu",    # slot 1 — pelvis / sacrum
    "femur_r_imu",   # slot 2 — right thigh
    "tibia_r_imu",   # slot 3 — right shank
    "calcn_r_imu",   # slot 4 — right foot / heel
    "femur_l_imu",   # slot 5 — left thigh
    "tibia_l_imu",   # slot 6 — left shank
    "calcn_l_imu",   # slot 7 — left foot / heel
]
N_SENSORS = len(SENSORS)

# SENSOR_CHAIN_UP: the physical attachment order on the body from distal
# (foot) upward to torso.  When fewer than 8 sensors are connected, the
# firmware assigns them starting from the most distal sensor (index 0 = shin)
# and works upward.  This list maps physical RP slot 0,1,2,... to OpenSim
# frame names for that reduced-sensor scenario.
SENSOR_CHAIN_UP = [
    "tibia_r_imu",   # physical slot 0 = right shin (most common single-sensor use)
    "femur_r_imu",   # physical slot 1 = right thigh
    "pelvis_imu",    # physical slot 2 = pelvis
    "torso_imu",     # physical slot 3 = trunk
    "calcn_r_imu",   # physical slot 4 = right foot
    "femur_l_imu",   # physical slot 5 = left thigh
    "tibia_l_imu",   # physical slot 6 = left shin
    "calcn_l_imu",   # physical slot 7 = left foot
]


def _sensor_names_for_count(n_sensors):
    n = max(0, int(n_sensors))
    if n <= 0:
        return []
    if n <= len(SENSOR_CHAIN_UP):
        return list(SENSOR_CHAIN_UP[:n])
    return list(SENSOR_CHAIN_UP) + [f"sensor_{i}_imu" for i in range(len(SENSOR_CHAIN_UP), n)]


def _slot_map_for_sensor_count(n_sensors):
    names = _sensor_names_for_count(n_sensors)
    slots = []
    for name in names:
        if name not in SENSORS:
            return None
        slots.append(SENSORS.index(name))
    return slots


def _neutral_quats_opensim_frame():
    return list(_NEUTRAL_QUATS_OPENSIM)


def _merge_live_quats_with_neutral(live_quats, live_sensor_names):
    full = _neutral_quats_opensim_frame()
    for q, name in zip(live_quats, live_sensor_names):
        if name in SENSORS:
            full[SENSORS.index(name)] = q
    return full

_NEUTRAL_QUATS_8IMU = [
    [0.0322464008, 0.8097193166, 0.0247206551, 0.5854089914],
    [0.0185830513, 0.7995226470, 0.0312063376, 0.5995367976],
    [0.5671632980, -0.3985867099, -0.5577065398, -0.4565280316],
    [0.5507267070, -0.4935519648, -0.5463582378, -0.3931910836],
    [-0.2254915660, 0.7089713644, 0.6466146264, -0.1685309557],
    [0.5169978228, 0.5582198006, -0.4095326078, 0.5033755542],
    [0.4013054075, 0.6224107200, -0.4017649263, 0.5386499879],
    [0.1921342610, 0.7097108826, -0.6558419822, -0.1710736192],
]

_frame_lock = threading.Lock()
_frame_queue = []
_udp_running = True
_angle_feedback_sock = None

_display_filter_lock = threading.Lock()
_display_filter_joints = [opensim_joint_catalog.DEFAULT_DISPLAY_JOINT]
_display_filter_seq = -1
_joint_display_watcher_running = True


def _qx(theta):
    h = theta / 2.0
    return [math.cos(h), math.sin(h), 0.0, 0.0]


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]


def _quat_conj(q):
    w, x, y, z = q
    return [w, -x, -y, -z]


def _quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [w / n, x / n, y / n, z / n]


def _quat_dot(q1, q2):
    return sum(a * b for a, b in zip(_quat_normalize(q1), _quat_normalize(q2)))


def _quat_distance(q1, q2):
    return 1.0 - min(1.0, abs(_quat_dot(q1, q2)))


# ── Accel/Gyro baseline calibration state ────────────────────────────────────
# Every IMU chip has a tiny constant error called "bias" or "zero-g offset":
# even when perfectly still, the sensor reports a small non-zero value.
# We measure this during the first CALIB_DURATION_S (3 s) of each session
# while the subject stands still, then subtract it from all future readings.
#
# Without this correction, the AHRS would accumulate orientation error at
# roughly 1-2°/min from the gyro bias alone.
#
# Storage layout: _ag_offsets[slot] = [ax, ay, az, gx, gy, gz]
#   where slot indexes into SENSORS (0=torso … 7=calcn_l).
_ag_calib_lock    = threading.Lock()
_ag_offsets       = None     # None = calibration not yet complete
_ag_calib_session = None     # session ID for which we are accumulating
_ag_calib_t_end   = 0.0      # wall-clock deadline for accumulation
_ag_calib_sums    = None     # running sums of raw values per slot
_ag_calib_counts  = None     # number of samples accumulated per slot


def _zero_ag_offsets():
    """Return a list of per-slot zero offsets (all six channels = 0)."""
    return [[0.0] * 6 for _ in range(N_SENSORS)]


def _reset_ag_calib_for_session(session_id):
    """Start a fresh 3 s still capture for a new streaming session.

    The subject should stand still during this window.  Once it expires,
    _accumulate_ag_calib will compute the averages and store them in
    _ag_offsets, after which _apply_ag_offsets subtracts them from every
    subsequent packet.
    """
    global _ag_offsets, _ag_calib_session, _ag_calib_t_end, _ag_calib_sums, _ag_calib_counts
    with _ag_calib_lock:
        _ag_calib_session = session_id
        _ag_calib_t_end = time.time() + CALIB_DURATION_S
        _ag_offsets = None
        _ag_calib_sums = [[0.0] * 6 for _ in range(N_SENSORS)]
        _ag_calib_counts = [0] * N_SENSORS
    print(
        f"[CALIB] Accel/gyro baseline — hold standing still for {CALIB_DURATION_S:.0f} s "
        f"(session {session_id})"
    )


def _accumulate_ag_calib(raw_values, slot_map):
    """Return True once baseline offsets are ready for this session."""
    global _ag_offsets, _ag_calib_t_end, _ag_calib_sums, _ag_calib_counts
    with _ag_calib_lock:
        if _ag_offsets is not None:
            return True

        for slot in slot_map:
            base = slot * 6
            for k in range(6):
                _ag_calib_sums[slot][k] += raw_values[base + k]
            _ag_calib_counts[slot] += 1

        if time.time() < _ag_calib_t_end:
            return False

        _ag_offsets = _zero_ag_offsets()
        for slot in slot_map:
            count = _ag_calib_counts[slot]
            if count <= 0:
                continue
            inv = 1.0 / count
            for k in range(6):
                _ag_offsets[slot][k] = _ag_calib_sums[slot][k] * inv

    print("[CALIB] Accel/gyro baseline complete — subtracting from live stream")
    for slot in slot_map:
        off = _ag_offsets[slot]
        print(
            f"[CALIB]   {SENSORS[slot]}: "
            f"acc=[{off[0]:.4f},{off[1]:.4f},{off[2]:.4f}] "
            f"gyro=[{off[3]:.4f},{off[4]:.4f},{off[5]:.4f}]"
        )
    return True


def _get_ag_offsets():
    with _ag_calib_lock:
        return _ag_offsets


def _apply_ag_offsets(values, offsets):
    if not offsets:
        return values
    corrected = list(values)
    for slot in range(N_SENSORS):
        off = offsets[slot]
        if all(abs(v) < 1e-12 for v in off):
            continue
        base = slot * 6
        for k in range(6):
            corrected[base + k] -= off[k]
    return corrected


# ── OpenSim coordinate-frame rotation ────────────────────────────────────────
# The Red Pitaya IMU firmware outputs quaternions in its own sensor frame.
# OpenSim uses a different convention for its "world" frame (Y-up vs Z-up).
# _Q_OPENSIM_FRAME is a -90° rotation around the X-axis that transforms from
# the firmware's frame into the OpenSim IK solver's expected frame.
# Every quaternion passed to the IK solver must be pre-multiplied by this.
_Q_OPENSIM_FRAME = _qx(-math.pi / 2.0)  # [cos(-45°), sin(-45°), 0, 0]

# Pre-rotate the hardcoded neutral pose quaternions into the OpenSim frame
# so they can be used directly to assemble the initial neutral skeleton pose.
_NEUTRAL_QUATS_OPENSIM = [_quat_mul(_Q_OPENSIM_FRAME, q) for q in _NEUTRAL_QUATS_8IMU]

_STRUCT_1F = struct.Struct("<f")
_STRUCT_3F = struct.Struct("<3f")
_STRUCT_4F = struct.Struct("<4f")
_floats_struct_cache = {}


def _floats_struct(n):
    s = _floats_struct_cache.get(n)
    if s is None:
        s = struct.Struct(f"<{n}f")
        _floats_struct_cache[n] = s
    return s



def _is_flat_fake_packet(values):
    for i in range(8):
        base = i * 6
        ax, ay, az = values[base], values[base + 1], values[base + 2]
        gx, gy, gz = values[base + 3], values[base + 4], values[base + 5]
        if abs(ax) > 1e-5 or abs(ay) > 1e-5 or abs(az - 1.0) > 1e-5:
            return False
        if abs(gx) > 1e-5 or abs(gy) > 1e-5 or abs(gz) > 1e-5:
            return False
    return True




def _load_display_joint(reload=False):
    global _display_joint_cache, _display_joint_mtime
    path = os.path.join(WORK_DIR, "opensim_display_joint.json")
    if reload:
        try:
            mtime = os.path.getmtime(path) if os.path.isfile(path) else None
        except OSError:
            mtime = None
        # Only re-parse the JSON when the file actually changed.
        if mtime != _display_joint_mtime or _display_joint_cache is None:
            _display_joint_mtime = mtime
            _display_joint_cache = None
    if _display_joint_cache is not None:
        return _display_joint_cache

    default = JOINT_OPTIONS[1]
    joint_name = default[0]
    joint_label = default[1]
    joint_index = default[2]

    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data.get("joint"), str):
                joint_name = data["joint"]
            if isinstance(data.get("label"), str):
                joint_label = data["label"]
            if "joint_index" in data:
                joint_index = int(data["joint_index"])
        except Exception as exc:
            print(f"[WARN] Could not read {path}: {exc}")

    for name, label, index in JOINT_OPTIONS:
        if name == joint_name:
            joint_label = label
            joint_index = index
            break

    _display_joint_cache = {
        "joint": joint_name,
        "label": joint_label,
        "joint_index": joint_index,
    }
    return _display_joint_cache


def _read_coord_value(model, state, joint_name):
    """Read a joint coordinate value from the current IK state, in degrees.

    Accepts either a full OpenSim coordinate name ("knee_angle_r") or a
    HUD abbreviation ("knee_r") — the catalog resolves abbreviations first.

    Steps:
      1. Resolve abbreviation → full coordinate name via opensim_joint_catalog.
      2. Call model.realizePosition(state) to ensure the state is up-to-date.
      3. Find the Coordinate object in the model's CoordinateSet.
      4. Read the value in radians, convert to degrees.
    Returns float("nan") if anything goes wrong (unknown name, IK failure, etc.).
    """
    coord_name = opensim_joint_catalog.coordinate_for(joint_name)
    try:
        model.realizePosition(state)
    except Exception:
        pass

    coord_set = model.getCoordinateSet()
    coord = None
    try:
        coord = coord_set.get(coord_name)
    except Exception:
        for i in range(coord_set.getSize()):
            candidate = coord_set.get(i)
            if candidate.getName() == coord_name:
                coord = candidate
                break
    if coord is None:
        return float("nan")
    try:
        value = float(coord.getValue(state))
        if not math.isfinite(value):
            return float("nan")
        return float(math.degrees(value))
    except Exception:
        return float("nan")


def _send_angle_feedback(t_stream, joint_index, angle_deg):
    """Send the selected joint angle back to the Open Ephys plugin on UDP port 5001.

    Packet format (4 × float32, little-endian):
      [0] t_stream    — stream timestamp in seconds since session start
      [1] 3.1         — version tag (UDP_PACKET_VERSION_ANGLES)
      [2] joint_index — which joint (matches JOINT_OPTIONS index)
      [3] angle_deg   — computed angle in degrees (or NaN on failure)

    The plugin optionally reads this port to display the real-time angle in
    the Open Ephys UI or to log it alongside neural data.
    """
    global _angle_feedback_sock
    if _angle_feedback_sock is None:
        try:
            _angle_feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as exc:
            print(f"[WARN] Angle feedback socket unavailable: {exc}")
            return

    pkt = _STRUCT_4F.pack(
        float(t_stream),
        UDP_PACKET_VERSION_ANGLES,
        float(joint_index),
        float(angle_deg) if math.isfinite(angle_deg) else float("nan"),
    )
    try:
        _angle_feedback_sock.sendto(pkt, (ANGLE_FEEDBACK_IP, ANGLE_FEEDBACK_PORT))
    except OSError:
        pass


def _format_angle_hud(joint_label, angle_deg):
    if angle_deg is None or not math.isfinite(angle_deg):
        return f"{joint_label}: --.- deg"
    return f"{joint_label}: {angle_deg:.1f} deg"


_window_title_supported = None
_window_title_warned = False


def _try_set_window_title(viz, title):
    """OpenSim 4.5 SWIG rejects plain Python str for setWindowTitle; skip after first failure."""
    global _window_title_supported, _window_title_warned
    if _window_title_supported is False:
        return
    try:
        viz.setWindowTitle(title)
        _window_title_supported = True
    except Exception as exc:
        _window_title_supported = False
        if not _window_title_warned:
            _window_title_warned = True
            print(
                f"[WARN] setWindowTitle unavailable ({exc}); "
                "using in-viewport DecorativeText HUD only"
            )


def _try_enable_model_geometry(model, simbody_viz):
    """Enable mesh display where OpenSim 4.5 Python bindings expose an API."""
    model_viz = model.getVisualizer()
    enabled = False
    for target, method, value in (
        (model_viz, "setShowGeometry", True),
        (model_viz, "setShowMarkers", True),
        (simbody_viz, "setShowFrameGeometry", False),
        (simbody_viz, "setShowWrapGeometry", False),
    ):
        if not hasattr(target, method):
            continue
        try:
            getattr(target, method)(value)
            if "Geometry" in method or method == "setShowMarkers":
                enabled = True
        except Exception:
            pass
    if enabled:
        print("[VIZ] Model geometry + markers enabled.")
    else:
        print("[VIZ] Geometry API unavailable on this OpenSim build; meshes load from Geometry/ if present.")


def _read_joint_display_config():
    path = os.path.join(WORK_DIR, JOINT_DISPLAY_CONFIG)
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        print(f"[WARN] Could not read {path}: {exc}")
        return None
    joints = data.get("joints")
    seq = data.get("seq")
    if not isinstance(joints, list) or not isinstance(seq, int):
        return None
    validated = opensim_joint_catalog.validate_joints(joints)
    if not validated:
        validated = [opensim_joint_catalog.DEFAULT_DISPLAY_JOINT]
    return seq, validated


def _joint_display_watcher_thread():
    """File-watcher thread: polls opensim_joint_display_config.json every 50 ms.

    When the file changes (detected via OS modification timestamp), reads the
    JSON and updates _display_filter_joints under _display_filter_lock.

    The 'seq' field in the JSON is a monotonically increasing counter written
    by the Open Ephys plugin.  We only apply updates where seq > last seen,
    so stale file writes (e.g. on reconnect) can never revert a newer selection.

    Why a file instead of a UDP command?
    Writing a JSON sidecar requires no protocol changes and is atomic on all
    major file systems.  The 50 ms poll latency is imperceptible for a trigger.
    """
    global _display_filter_joints, _display_filter_seq
    path = os.path.join(WORK_DIR, JOINT_DISPLAY_CONFIG)
    last_mtime = None
    while _joint_display_watcher_running:
        try:
            mtime = os.path.getmtime(path) if os.path.isfile(path) else None
        except OSError:
            mtime = None
        if mtime is not None and mtime != last_mtime:
            last_mtime = mtime
            result = _read_joint_display_config()
            if result is not None:
                seq, validated = result
                with _display_filter_lock:
                    if seq > _display_filter_seq:
                        _display_filter_seq = seq
                        _display_filter_joints = list(validated)
                        print(f"[JOINT-DISPLAY] seq={seq} joints={validated}")
        time.sleep(0.05)


def get_display_filter_joints():
    with _display_filter_lock:
        joints = list(_display_filter_joints)
    if not joints:
        joints = [opensim_joint_catalog.DEFAULT_DISPLAY_JOINT]
    return joints


_HUD_BASE_TITLE = "Connect OpenSim - RedPitaya 8-IMU"
_last_hud_text = None
_hud_screen_text = None
_HUD_SCREEN_COLOR = osim.Vec3(1.0, 0.55, 0.0)
_HUD_SCREEN_SCALE = osim.Vec3(0.025, 0.025, 0.025)


def _hud_screen_transform():
    """Normalized viewport coords; place joint block under Simbody sim-time readout."""
    xform = osim.Transform()
    xform.setP(osim.Vec3(-0.92, 0.52, 0.0))
    return xform


def _build_joint_hud_lines(model, state, last_known=None):
    """Read angle values for every selected joint and format them as HUD lines.

    filter_joints comes from _display_filter_joints (updated by the watcher
    thread).  For each joint:
      - Read the current IK angle via _read_coord_value.
      - If reading fails (NaN), fall back to the last known good value.
      - Format as "abbrev: XX.X°".

    last_known is a dict {coord_name: degrees} that persists the last valid
    angle so the HUD does not go blank during transient IK failures.
    """
    lines = []
    filter_joints = get_display_filter_joints()
    for name in filter_joints:
        degrees = _read_coord_value(model, state, name)
        coord_name = opensim_joint_catalog.coordinate_for(name)
        if not math.isfinite(degrees) and last_known is not None:
            cached = last_known.get(coord_name)
            if cached is not None and math.isfinite(cached):
                degrees = cached
        abbrev = opensim_joint_catalog.abbrev_for(coord_name)
        if not math.isfinite(degrees):
            lines.append(f"{abbrev}: --.--°")
            continue
        lines.append(opensim_joint_catalog.format_angle_line(abbrev, degrees))
        if last_known is not None:
            last_known[coord_name] = degrees
    return lines


def _init_screen_text_hud(viz):
    global _hud_screen_text
    xform = _hud_screen_transform()
    text = osim.DecorativeText("knee_r: --.--°")
    text.setIsScreenText(True)
    text.setColor(_HUD_SCREEN_COLOR)
    text.setScaleFactors(_HUD_SCREEN_SCALE)
    text.setTransform(xform)
    decor_idx = viz.addDecoration(0, xform, text)
    _hud_screen_text = text
    print(f"[JOINT-DISPLAY] in-viewport screen text enabled (decoration idx={decor_idx})")
    return True


def _pick_hud_strategy(viz):
    # Prefer the window title because Simbody copies DecorativeText decorations
    # on addDecoration(), so mutating the Python object can leave stale text.
    print("[JOINT-DISPLAY-SPIKE] chosen strategy=window_title (screen text disabled)")
    return "window_title"


def _render_joint_display_hud(model, viz, state, strategy, base_title=_HUD_BASE_TITLE, last_known=None):
    global _last_hud_text
    lines = _build_joint_hud_lines(model, state, last_known)
    if not lines:
        return

    compact = " | ".join(lines)
    text_changed = compact != _last_hud_text
    _last_hud_text = compact

    if strategy == "screen_text" and _hud_screen_text is not None:
        try:
            if text_changed:
                _hud_screen_text.setText(compact)
        except Exception as exc:
            print(f"[WARN] screen text HUD update failed: {exc}")
        return

    display_text = f"{base_title} | {compact}"
    if text_changed:
        _try_set_window_title(viz, display_text)


def _write_test_joint_display_config(joints, seq):
    path = os.path.join(WORK_DIR, JOINT_DISPLAY_CONFIG)
    payload = {
        "joints": list(joints),
        "trigger_ts": time.time(),
        "seq": int(seq),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")


def _load_sensor_map():
    global _sensor_map_cache
    if _sensor_map_cache is not None:
        return _sensor_map_cache
    path = os.path.join(WORK_DIR, "opensim_sensor_map.json")
    if not os.path.isfile(path):
        _sensor_map_cache = {}
        return _sensor_map_cache
    try:
        with open(path, "r", encoding="utf-8") as f:
            _sensor_map_cache = json.load(f)
    except Exception as exc:
        print(f"[WARN] Could not read {path}: {exc}")
        _sensor_map_cache = {}
    return _sensor_map_cache


def _slot_map_for_quat_packet(n_sensors, sensor_map=None):
    if sensor_map is None:
        sensor_map = _load_sensor_map()
    names = sensor_map.get("sensor_slots")
    if isinstance(names, list) and len(names) >= n_sensors:
        slots = []
        for name in names[:n_sensors]:
            if name not in SENSORS:
                return None
            slots.append(SENSORS.index(name))
        return slots
    return _slot_map_for_sensor_count(n_sensors)


def _parse_imu_v3_packet(data):
    n_floats = len(data) // 4
    if n_floats < 3:
        return None
    t0, ver, n_s = _STRUCT_3F.unpack(data[:12])
    if not (2.99 < ver < 3.01):
        return None
    n_sensors = int(round(n_s))
    if n_sensors < 1 or n_sensors > N_SENSORS:
        return None
    expected = 3 + n_sensors * 6
    if n_floats != expected:
        return None
    imu = _floats_struct(n_sensors * 6).unpack(data[12:12 + n_sensors * 24])
    return t0, n_sensors, imu


def _parse_quat_v2_packet(data):
    n_floats = len(data) // 4
    if n_floats < 3:
        return None
    t0, ver, n_s = _STRUCT_3F.unpack(data[:12])
    if not (1.99 < ver < 2.01):
        return None
    n_sensors = int(round(n_s))
    if n_sensors < 1 or n_sensors > N_SENSORS:
        return None
    expected = 3 + n_sensors * 4
    if n_floats != expected:
        return None
    quats = _floats_struct(n_sensors * 4).unpack(data[12:12 + n_sensors * 16])
    return t0, n_sensors, quats

def _slot_map_for_packet_imus(n_imus):
    return _slot_map_for_sensor_count(n_imus)


def _expand_imu_packet_to_8_slots(raw_values, slot_map):
    values = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0] * N_SENSORS
    for packet_i, slot_i in enumerate(slot_map):
        src = packet_i * 6
        dst = slot_i * 6
        values[dst:dst + 6] = raw_values[src:src + 6]
    return values



def _active_imu_summary(values, slot_map, previous_values=None):
    parts = []
    max_gyro_norm = 0.0
    max_change = 0.0

    for slot in slot_map:
        base = slot * 6
        ax, ay, az = values[base], values[base + 1], values[base + 2]
        gx, gy, gz = values[base + 3], values[base + 4], values[base + 5]
        accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
        gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)

        if accel_norm < 0.3 or accel_norm > 3.0:
            parts.append(f"slot{slot}({SENSORS[slot]}) INVALID accel_norm={accel_norm:.1f}")
            continue  # exclude garbage slots from static detection

        max_gyro_norm = max(max_gyro_norm, gyro_norm)

        if previous_values is None:
            change = 0.0
        else:
            change = max(abs(values[base + k] - previous_values[base + k]) for k in range(6))
        max_change = max(max_change, change)

        parts.append(
            f"slot{slot}({SENSORS[slot]}) "
            f"acc=[{ax:.3f},{ay:.3f},{az:.3f}]|norm={accel_norm:.3f} "
            f"gyro=[{gx:.2f},{gy:.2f},{gz:.2f}]|norm={gyro_norm:.2f} "
            f"change={change:.3f}"
        )

    active_static = max_gyro_norm < STATIC_GYRO_EPS and max_change < STATIC_CHANGE_EPS
    return " ; ".join(parts), active_static, max_gyro_norm, max_change



def _udp_ahrs_thread():
    """UDP receiver + AHRS fusion thread (runs as a daemon thread).

    Lifecycle:
      - Bind UDP port 5000.
      - Loop forever reading packets until _udp_running is set False.
      - For each packet:
          1. Drain the socket buffer (take only the newest frame if multiple
             arrived since last check — prevents the visualiser from lagging
             behind when IK is slow).
          2. Detect packet format (v3 / v2 / legacy).
          3. Detect session reset (timestamp jump, source change, long gap).
          4. Run accel/gyro baseline calibration for the first 3 s.
          5. Run imufusion.Ahrs per sensor (v3/legacy only) → quaternions.
          6. Apply neutral-pose correction and OpenSim frame rotation.
          7. Push result to _frame_queue (replace previous frame, not append).

    Session reset resets: AHRS filters, calibration, frame queue, counters.
    A "session" corresponds to one continuous streaming run from Open Ephys.
    """
    ahrs_list = [imufusion.Ahrs() for _ in range(N_SENSORS)]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except OSError as e:
        print(f"\n{'='*60}")
        print(f"[ERROR] Cannot bind UDP port {UDP_PORT}: {e}")
        print(f"  Another process is already using port {UDP_PORT}.")
        print(f"  Fix: run this command in PowerShell to find and kill it:")
        print(f"    $p = (netstat -ano | Select-String ':{UDP_PORT}.*LISTENING' | Select-Object -First 1 | ForEach-Object {{ ($_.ToString().Trim() -split '\\s+')[-1] }}); if ($p) {{ Stop-Process -Id $p -Force; Write-Host 'Killed PID' $p }} else {{ Write-Host 'Port free' }}")
        print(f"{'='*60}\n")
        sock.close()
        return
    sock.settimeout(1.0)
    print(f"[START] OpenSim live bridge starting...")
    print(f"[WAITING] Waiting for LFP Viewer/Open Ephys IMU packets on port {UDP_PORT}...")
    print(f"[UDP] Listening on port {UDP_PORT} for {N_SENSORS}-IMU packets ...")
    _t_last_pkt = None
    _pkt_n = 0
    first_packet_timestamp = None
    last_packet_timestamp = None
    session_id = 0
    last_source_key = None
    last_expanded_values = None
    neutral_static_since = None
    neutral_warned = False
    _connected_announced = False

    while _udp_running:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue

        drained = 0
        while True:
            readable, _, _ = select.select([sock], [], [], 0)
            if not readable:
                break
            try:
                data, _ = sock.recvfrom(4096)
                drained += 1
            except socket.timeout:
                break

        now = time.perf_counter()
        if _t_last_pkt is None:
            dt = 1.0 / SAMPLE_RATE
        else:
            dt = now - _t_last_pkt
        _t_last_pkt = now

        n_bytes = len(data)
        n_floats_total = n_bytes // 4

        imu_v3 = _parse_imu_v3_packet(data)
        if imu_v3 is not None:
            packet_timestamp, n_sensors, imu_floats = imu_v3
            slot_map = _slot_map_for_quat_packet(n_sensors)
            if slot_map is None:
                continue
            source_key = (SOURCE_LABEL, "imu_v3", n_sensors, tuple(slot_map))
            packet_imus = n_sensors
            raw_values = _expand_imu_packet_to_8_slots(list(imu_floats), slot_map)

            new_session = False
            reason = ""
            if last_source_key is not None and source_key != last_source_key:
                new_session = True
                reason = f"source/slot changed {last_source_key}->{source_key}"
            elif first_packet_timestamp is None or last_packet_timestamp is None:
                new_session = True
                reason = "first packet"
            elif packet_timestamp < last_packet_timestamp - 0.05:
                new_session = True
                reason = f"timestamp reset {last_packet_timestamp:.3f}->{packet_timestamp:.3f}"
            elif dt > SESSION_GAP_S:
                new_session = True
                reason = f"packet gap {dt*1000.0:.1f} ms"

            if new_session:
                session_id += 1
                first_packet_timestamp = packet_timestamp
                last_packet_timestamp = packet_timestamp
                ahrs_list = [imufusion.Ahrs() for _ in range(N_SENSORS)]
                with _frame_lock:
                    _frame_queue.clear()
                dt = 1.0 / SAMPLE_RATE
                last_source_key = source_key
                last_expanded_values = None
                neutral_static_since = None
                neutral_warned = False
                _connected_announced = False
                if not OPENSIM_SKIP_CALIB:
                    _reset_ag_calib_for_session(session_id)
                print(f"[SESSION] new IMU v3 session id={session_id} reason={reason}")
            else:
                dt = max(1.0 / SAMPLE_RATE, packet_timestamp - last_packet_timestamp)
                last_packet_timestamp = packet_timestamp

            if not _connected_announced:
                _connected_announced = True
                active_names = [SENSORS[s] for s in slot_map]
                print(f"[CONNECTED] Receiving OpenSim UDP v3 accel/gyro packets.")
                print(f"[SENSORS] {n_sensors} sensor(s): {', '.join(active_names)}")
                for idx, name in zip(slot_map, active_names):
                    print(f"[MAP] sensor{idx} → {name}")

            active_summary, active_static, active_gyro_norm, active_change = _active_imu_summary(
                raw_values, slot_map, last_expanded_values
            )
            last_expanded_values = list(raw_values)

            if not OPENSIM_SKIP_CALIB and not _accumulate_ag_calib(raw_values, slot_map):
                continue

            values = _apply_ag_offsets(raw_values, _get_ag_offsets())

            _pkt_n += 1
            t_stream = max(0.0, packet_timestamp - first_packet_timestamp)
            if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
                print(
                    f"[PKT v3 {_pkt_n}] t={t_stream:.3f}s sensors={n_sensors} "
                    f"slots={slot_map} static={active_static}"
                )

            active_quats = []
            active_sensors = []
            for i in range(N_SENSORS):
                if i not in slot_map:
                    continue
                base = i * 6
                ax, ay, az = values[base], values[base + 1], values[base + 2]
                gx, gy, gz = values[base + 3], values[base + 4], values[base + 5]
                ahrs_list[i].update_no_magnetometer(
                    np.array([gx, gy, gz], dtype=np.float64),
                    np.array([ax, ay, az], dtype=np.float64),
                    dt,
                )
                accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
                if accel_norm < 0.3 or accel_norm > 3.0:
                    continue
                q = ahrs_list[i].quaternion
                nw, nx, ny, nz = _NEUTRAL_QUATS_8IMU[i]
                qw, qx, qy, qz = q.w, q.x, q.y, q.z
                _q_raw = [
                    nw * qw - nx * qx - ny * qy - nz * qz,
                    nw * qx + nx * qw + ny * qz - nz * qy,
                    nw * qy - nx * qz + ny * qw + nz * qx,
                    nw * qz + nx * qy - ny * qx + nz * qw,
                ]
                active_quats.append(_quat_mul(_Q_OPENSIM_FRAME, _q_raw))
                active_sensors.append(SENSORS[i])

            if not active_quats:
                continue

            with _frame_lock:
                _frame_queue.clear()
                _frame_queue.append(
                    (t_stream, active_quats, active_sensors, session_id, now, active_static, active_gyro_norm, active_change)
                )
            continue

        quat_pkt = _parse_quat_v2_packet(data)
        if quat_pkt is not None:
            packet_timestamp, n_sensors, quat_floats = quat_pkt
            slot_map = _slot_map_for_quat_packet(n_sensors)
            if slot_map is None:
                continue
            source_key = (SOURCE_LABEL, "quat_v2", n_sensors, tuple(slot_map))
            packet_imus = n_sensors

            new_session = False
            reason = ""
            if last_source_key is not None and source_key != last_source_key:
                new_session = True
                reason = f"source/slot changed {last_source_key}->{source_key}"
            elif first_packet_timestamp is None or last_packet_timestamp is None:
                new_session = True
                reason = "first packet"
            elif packet_timestamp < last_packet_timestamp - 0.05:
                new_session = True
                reason = f"timestamp reset {last_packet_timestamp:.3f}->{packet_timestamp:.3f}"
            elif dt > SESSION_GAP_S:
                new_session = True
                reason = f"packet gap {dt*1000.0:.1f} ms"

            if new_session:
                session_id += 1
                first_packet_timestamp = packet_timestamp
                last_packet_timestamp = packet_timestamp
                with _frame_lock:
                    _frame_queue.clear()
                dt = 1.0 / SAMPLE_RATE
                last_source_key = source_key
                last_expanded_values = None
                neutral_static_since = None
                neutral_warned = False
                _connected_announced = False
                print(f"[SESSION] new quat v2 session id={session_id} reason={reason}")
                print(
                    "[CALIB] WARN: UDP v2 quaternions carry no raw accel/gyro; "
                    "Python a,g baseline calib skipped (use UDP v3 from plugin or legacy IMU packets)"
                )
            else:
                dt = max(1.0 / SAMPLE_RATE, packet_timestamp - last_packet_timestamp)
                last_packet_timestamp = packet_timestamp

            if not _connected_announced:
                _connected_announced = True
                active_names = [SENSORS[s] for s in slot_map]
                print(f"[CONNECTED] Receiving OpenSim UDP v2 quaternion packets.")
                print(f"[SENSORS] {n_sensors} sensor(s): {', '.join(active_names)}")
                for idx, name in zip(slot_map, active_names):
                    print(f"[MAP] sensor{idx} → {name}")

            active_quats = []
            active_sensors = []
            for pkt_i, slot_i in enumerate(slot_map):
                base = pkt_i * 4
                qw, qx, qy, qz = quat_floats[base:base + 4]
                active_quats.append(_quat_mul(_Q_OPENSIM_FRAME, [qw, qx, qy, qz]))
                active_sensors.append(SENSORS[slot_i])

            if not active_quats:
                continue

            _pkt_n += 1
            t_stream = max(0.0, packet_timestamp - first_packet_timestamp)
            if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
                print(f"[PKT v2 {_pkt_n}] t={t_stream:.3f}s sensors={n_sensors} slots={slot_map}")
            with _frame_lock:
                _frame_queue.clear()
                _frame_queue.append((t_stream, active_quats, active_sensors, session_id, now, False, 0.0, 0.0))
            continue

        # Disambiguate packet format:
        #   With-timestamp (Open Ephys):  (n_floats_total - 1) % 6 == 0
        #     e.g. 196 bytes → 49 floats → 1 ts + 48 IMU = 8 IMUs
        #   No-timestamp (legacy):         n_floats_total % 6 == 0
        #     e.g. 192 bytes → 48 floats → 8 IMUs, IMU0.ax starts at byte 0
        # These two conditions can never both be true (consecutive integers).
        if n_floats_total >= 2 and (n_floats_total - 1) % 6 == 0:
            packet_timestamp = _STRUCT_1F.unpack(data[:4])[0]
            imu_start = 4
            n_imus = (n_floats_total - 1) // 6
        elif n_floats_total >= 6 and n_floats_total % 6 == 0:
            packet_timestamp = _pkt_n / SAMPLE_RATE
            imu_start = 0
            n_imus = n_floats_total // 6
        else:
            continue

        if n_imus < 1:
            continue

        packet_imus = min(n_imus, N_SENSORS)
        slot_map = _slot_map_for_packet_imus(packet_imus)
        if slot_map is None:
            continue
        source_key = (SOURCE_LABEL, packet_imus, tuple(slot_map))

        new_session = False
        reason = ""
        if last_source_key is not None and source_key != last_source_key:
            new_session = True
            reason = f"source/slot changed {last_source_key}->{source_key}"
        elif _t_last_pkt is None or first_packet_timestamp is None or last_packet_timestamp is None:
            new_session = True
            reason = "first packet"
        elif packet_timestamp < last_packet_timestamp - 0.05:
            new_session = True
            reason = f"timestamp reset {last_packet_timestamp:.3f}->{packet_timestamp:.3f}"
        elif dt > SESSION_GAP_S:
            new_session = True
            reason = f"packet gap {dt*1000.0:.1f} ms"

        if new_session:
            session_id += 1
            first_packet_timestamp = packet_timestamp
            last_packet_timestamp = packet_timestamp
            ahrs_list = [imufusion.Ahrs() for _ in range(N_SENSORS)]
            with _frame_lock:
                _frame_queue.clear()
            dt = 1.0 / SAMPLE_RATE
            last_source_key = source_key
            last_expanded_values = None
            neutral_static_since = None
            neutral_warned = False
            _connected_announced = False
            if not OPENSIM_SKIP_CALIB:
                _reset_ag_calib_for_session(session_id)
            if "source/slot changed" in reason:
                print(f"[SESSION] source changed, reset AHRS id={session_id} reason={reason}")
            print(f"[SESSION] new play session detected id={session_id} reason={reason}")
        else:
            dt = max(1.0 / SAMPLE_RATE, packet_timestamp - last_packet_timestamp)
            last_packet_timestamp = packet_timestamp

        raw_values = _floats_struct(packet_imus * 6).unpack(
            data[imu_start:imu_start + packet_imus * 6 * 4]
        )
        values = _expand_imu_packet_to_8_slots(raw_values, slot_map)

        if not _connected_announced:
            _connected_announced = True
            active_names = [SENSORS[s] for s in slot_map]
            print(f"[CONNECTED] Receiving live IMU packets.")
            print(f"[SENSORS] Detected {packet_imus} IMU sensors: {', '.join(active_names)}")
            for idx, name in zip(slot_map, active_names):
                print(f"[MAP] sensor{idx} → {name}")

        if LIVE_MODE and not ALLOW_SAMPLE_DATA and _is_flat_fake_packet(values):
            if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
                print("[ERROR] Sample data blocked: perfectly neutral packet detected in LIVE_MODE.")
                print("  Reason: All IMU channels read (0,0,1,0,0,0) — matches fake/demo sender output.")
            _pkt_n += 1
            continue

        active_summary, active_static, active_gyro_norm, active_change = _active_imu_summary(
            values, slot_map, last_expanded_values
        )
        last_expanded_values = list(values)
        real_source = SOURCE_LABEL in REAL_SOURCE_LABELS or LIVE_MODE
        real_static_invalid = False

        if active_static:
            if neutral_static_since is None:
                neutral_static_since = now
            elif now - neutral_static_since > STATIC_NEUTRAL_S and not neutral_warned:
                print("[WARN] active real IMUs appear neutral/static")
                neutral_warned = True
            if real_source and neutral_static_since is not None and now - neutral_static_since > STATIC_NEUTRAL_S:
                real_static_invalid = True
        else:
            neutral_static_since = None
            neutral_warned = False

        _pkt_n += 1
        t_stream = max(0.0, packet_timestamp - first_packet_timestamp)
        if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
            print(f"[PKT {_pkt_n}] t={t_stream:.3f}s dt={dt:.4f}s imus={packet_imus} slots={slot_map} static={active_static} gyro_norm={active_gyro_norm:.2f} change={active_change:.3f}")
            if drained:
                print(f"[LIVE {_pkt_n}] drained {drained} stale UDP packets")
            print(f"[ACTIVE {_pkt_n}] {active_summary}")

        if real_static_invalid:
            if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
                print("[FREEZE] real data invalid/static; OpenSim update skipped")
            continue

        if not OPENSIM_SKIP_CALIB and not _accumulate_ag_calib(values, slot_map):
            continue

        values = _apply_ag_offsets(values, _get_ag_offsets())

        active_quats   = []
        active_sensors = []

        for i in range(N_SENSORS):
            if i not in slot_map:
                continue  # no real sensor in this slot — skip AHRS work and never feed filler to IK

            base = i * 6
            ax, ay, az = values[base], values[base + 1], values[base + 2]
            gx, gy, gz = values[base + 3], values[base + 4], values[base + 5]

            ahrs_list[i].update_no_magnetometer(
                np.array([gx, gy, gz], dtype=np.float64),
                np.array([ax, ay, az], dtype=np.float64),
                dt,
            )

            accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
            if accel_norm < 0.3 or accel_norm > 3.0:
                if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
                    print(f"[WARN] Slot {i} ({SENSORS[i]}) accel norm={accel_norm:.1f}g — invalid, skipping")
                continue

            q = ahrs_list[i].quaternion
            nw, nx, ny, nz = _NEUTRAL_QUATS_8IMU[i]
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            _q_raw = [
                nw*qw - nx*qx - ny*qy - nz*qz,
                nw*qx + nx*qw + ny*qz - nz*qy,
                nw*qy - nx*qz + ny*qw + nz*qx,
                nw*qz + nx*qy - ny*qx + nz*qw,
            ]
            active_quats.append(_quat_mul(_Q_OPENSIM_FRAME, _q_raw))
            active_sensors.append(SENSORS[i])

        if not active_quats:
            if _pkt_n <= 5 or _pkt_n % PACKET_LOG_INTERVAL == 0:
                print(f"[WARN] No valid sensor data in packet {_pkt_n} — all slots rejected by norm check. Model frozen.")
            continue

        if _pkt_n % PACKET_LOG_INTERVAL == 0:
            print(f"[DBG t={t_stream:.2f}s] active_sensors={active_sensors} n={len(active_quats)}")
        with _frame_lock:
            _frame_queue.clear()
            _frame_queue.append((t_stream, active_quats, active_sensors, session_id, now, active_static, active_gyro_norm, active_change))

    sock.close()


def _write_quat_sto(path, t, quats):
    with open(path, "w") as f:
        f.write("DataRate=100.0\n")
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.1\n")
        f.write("endheader\n")
        f.write("time\t" + "\t".join(SENSORS) + "\n")
        cols = "\t".join(
            f"{q[0]:.10f},{q[1]:.10f},{q[2]:.10f},{q[3]:.10f}" for q in quats
        )
        f.write(f"{t:.6f}\t{cols}\n")


def _quats_to_rot_table(t, quats, active_sensors):
    table = osim.TimeSeriesTableRotation()
    table.setColumnLabels(active_sensors)
    row = osim.RowVectorRotation(len(quats))
    for i, q in enumerate(quats):
        row.updElt(0, i).setRotationFromQuaternion(
            osim.Quaternion(q[0], q[1], q[2], q[3])
        )
    table.appendRow(t, row)
    return table


def _is_tibia_only_mode(active_sensors):
    return len(active_sensors) == 1 and active_sensors[0] == "tibia_r_imu"


TIBIA_ONLY_UNLOCKED_COORDS = {"knee_angle_r", "knee_angle_r_beta"}


def _set_coord_locked(coord, state, locked):
    try:
        coord.setLocked(state, locked)
        return
    except (AttributeError, TypeError):
        pass
    coord.set_locked(locked)


def _get_coord_locked(coord, state):
    try:
        return bool(coord.getLocked(state))
    except (AttributeError, TypeError):
        pass
    return bool(coord.get_locked())


def _capture_coordinate_locks(coord_set, state):
    locks = {}
    for i in range(coord_set.getSize()):
        coord = coord_set.get(i)
        locks[coord.getName()] = _get_coord_locked(coord, state)
    return locks


def _apply_tibia_only_locks(coord_set, state, enabled, default_locks):
    for i in range(coord_set.getSize()):
        coord = coord_set.get(i)
        name = coord.getName()
        should_lock = (name not in TIBIA_ONLY_UNLOCKED_COORDS) if enabled else default_locks.get(name, False)

        if enabled and should_lock:
            try:
                coord.setValue(state, coord.getDefaultValue())
            except Exception:
                pass

        _set_coord_locked(coord, state, should_lock)


def run_live():
    """Main entry point: load model, set up IK solver, run the visualisation loop.

    Sequence:
      1. Delete stale .sto files from previous runs.
      2. Load the OpenSim model (Rajagopal2015_opensense_calibrated.osim).
      3. Write a neutral-pose .sto file and assemble the initial IK state so
         the skeleton appears immediately before any sensor data arrives.
      4. Start _udp_ahrs_thread (receives UDP, pushes frames to queue).
      5. Start _joint_display_watcher_thread (watches the JSON sidecar).
      6. Wait up to MAX_WAIT_S for the first UDP frame.
      7. Main loop (up to LIVE_VISUALIZER_RATE Hz):
           a. Pop latest frame from _frame_queue.
           b. Build OrientationsReference → rebuild IK solver → assemble.
           c. Read selected joint angles → update HUD.
           d. model.getVisualizer().show(state).
      8. On KeyboardInterrupt: clean up sockets and threads.

    Note on IK solver rebuild:
      Ideally we would call ikSolver.track(state) to update orientations in
      place.  On this OpenSim 4.5 Python build, however, the solver's internal
      reference cache does not update between assemble() calls.  Rebuilding
      the solver from a fresh OrientationsReference each frame is the workaround
      that makes the skeleton actually follow the live sensor data.
    """
    for _stale in ("ephys_live_orientations.sto", "ephys_live_motion.sto", "_neutral_frame.sto"):
        _p = os.path.join(WORK_DIR, _stale)
        if os.path.exists(_p):
            os.remove(_p)
            print(f"[CLEANUP] Deleted stale file: {_stale}")
    print("[Connect OpenSim] Loading model ...")
    if not os.path.isfile(MODEL_PATH):
        print(f"[ERROR] Model file not found: {MODEL_PATH}")
        return
    model = osim.Model(MODEL_PATH)
    model.setUseVisualizer(True)

    neutral_sto = os.path.join(WORK_DIR, "_neutral_frame.sto")
    _write_quat_sto(neutral_sto, 0.0, _NEUTRAL_QUATS_OPENSIM)
    neutral_qt = osim.TimeSeriesTableQuaternion(neutral_sto)
    neutral_rt = osim.OpenSenseUtilities.convertQuaternionsToRotations(neutral_qt)

    mRefs = osim.MarkersReference()
    coordRefs = osim.SimTKArrayCoordinateReference()

    print("[Connect OpenSim] Initialising IK solver ...")
    state = model.initSystem()
    viz = model.updVisualizer().updSimbodyVisualizer()
    _try_enable_model_geometry(model, viz)
    oRefs = osim.OrientationsReference(neutral_rt)
    ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordRefs, CONSTRAINT)
    ikSolver.setAccuracy(1e-4)
    default_coord_locks = _capture_coordinate_locks(model.getCoordinateSet(), state)

    _try_set_window_title(viz, _HUD_BASE_TITLE)
    viz.setShowSimTime(True)
    hud_strategy = _pick_hud_strategy(viz)
    coord_set = model.getCoordinateSet()
    last_hud_angles = {}

    startup_cfg = _read_joint_display_config()
    if startup_cfg is not None:
        seq, validated = startup_cfg
        global _display_filter_joints, _display_filter_seq
        with _display_filter_lock:
            if seq > _display_filter_seq:
                _display_filter_seq = seq
                _display_filter_joints = list(validated)
                print(f"[JOINT-DISPLAY] startup seq={seq} joints={validated}")

    display_joint = _load_display_joint()
    print(f"[HUD] Display joint: {display_joint['label']} ({display_joint['joint']})")
    print(f"[HUD] Filter joints: {get_display_filter_joints()} strategy={hud_strategy}")

    print("[Connect OpenSim] Assembling neutral pose ...")
    ikSolver.assemble(state)
    _render_joint_display_hud(model, viz, state, hud_strategy, last_known=last_hud_angles)
    model.getVisualizer().show(state)
    print("[Connect OpenSim] Neutral pose assembled. Waiting for IMU stream ...")

    global _udp_running, _joint_display_watcher_running
    t_recv = threading.Thread(target=_udp_ahrs_thread, daemon=True)
    t_recv.start()
    t_joint_display = threading.Thread(target=_joint_display_watcher_thread, daemon=True)
    t_joint_display.start()

    t_start = time.time()
    while True:
        with _frame_lock:
            has = len(_frame_queue) > 0
        if has:
            break
        if time.time() - t_start > MAX_WAIT_S:
            print("[Connect OpenSim] Timeout - no UDP frames received. Exiting.")
            _udp_running = False
            return
        _render_joint_display_hud(model, viz, state, hud_strategy, last_known=last_hud_angles)
        model.getVisualizer().show(state)
        time.sleep(0.05)

    with _frame_lock:
        _frame_queue[-1]

    print("[Connect OpenSim] Streaming started. Close the Simbody window to stop.")

    last_t = -1.0
    last_session_id = None
    last_tibia_mode = None
    last_frame_wall_time = None
    last_age_print = 0.0
    skipped_by_throttle = 0
    last_coord_values = None
    live_frame = 0
    frame_times = []
    convert_times = []
    t_last_perf_print = time.time()
    t_last_solve = 0.0
    target_frame_s = 1.0 / LIVE_VISUALIZER_RATE
    target_frame_ms = target_frame_s * 1000.0

    try:
        while True:
            with _frame_lock:
                if _frame_queue:
                    t_imu, quats, active_sensors, session_id, packet_wall_time, active_static, active_gyro_norm, active_change = _frame_queue[-1]
                    _frame_queue.clear()
                else:
                    t_imu, quats, active_sensors, session_id, packet_wall_time = None, None, None, None, None
                    active_static, active_gyro_norm, active_change = False, 0.0, 0.0

            if t_imu is None:
                if last_frame_wall_time is not None:
                    age_s = time.perf_counter() - last_frame_wall_time
                    now_age = time.perf_counter()
                    if age_s > STALE_PACKET_S and now_age - last_age_print > 0.5:
                        print(f"[AGE] packet_age_ms={age_s*1000.0:.1f} freeze=true")
                        last_age_print = now_age
                    if age_s > NO_DATA_TIMEOUT_S and now_age - last_age_print > 2.0:
                        print(f"[ERROR] No live IMU data received for {age_s:.1f} seconds.")
                        print("  Reason: LFP Viewer/Open Ephys stopped sending packets or disconnected.")
                        print("  Model is frozen. Reconnect the sensor stream to resume.")
                        last_age_print = now_age
                _render_joint_display_hud(
                    model, viz, state, hud_strategy, last_known=last_hud_angles
                )
                model.getVisualizer().show(state)
                time.sleep(0.005)
                continue

            last_frame_wall_time = packet_wall_time
            tibia_only = _is_tibia_only_mode(active_sensors)
            if session_id != last_session_id:
                print(f"[SESSION] renderer reset id={session_id}")
                last_session_id = session_id
                last_t = -1.0
                t_last_solve = 0.0
                last_coord_values = None
                last_tibia_mode = None
                mode = "tibia-only knee lock" if tibia_only else "full active-sensor IK"
                print(f"[IK-MODE] {mode}")

            if t_imu <= last_t:
                time.sleep(0.005)
                continue

            now_solve = time.perf_counter()
            if now_solve - t_last_solve < target_frame_s:
                last_t = t_imu
                skipped_by_throttle += 1
                continue
            t_last_solve = now_solve
            t_frame0 = time.perf_counter()
            render_lag_ms = (t_frame0 - packet_wall_time) * 1000.0
            if live_frame <= 5 or live_frame % 60 == 0 or skipped_by_throttle:
                print(f"[LIVE-RENDER] frame={live_frame} opensim_t={t_imu:.3f} render_lag_ms={render_lag_ms:.1f} skipped_by_throttle={skipped_by_throttle}")
                skipped_by_throttle = 0

            t_convert0 = time.perf_counter()
            if tibia_only:
                ik_quats = quats
                ik_sensors = active_sensors
            else:
                ik_quats = (
                    quats
                    if active_sensors == SENSORS
                    else _merge_live_quats_with_neutral(quats, active_sensors)
                )
                ik_sensors = SENSORS
            rot_table = _quats_to_rot_table(0.0, ik_quats, ik_sensors)
            convert_times.append(time.perf_counter() - t_convert0)

            # BufferedOrientationsReference receives the changing rows, but on
            # this OpenSim 4.5 Python build the IK coordinates stayed fixed.
            # Rebuilding the reference from the latest labelled row makes the
            # solver use the current IMU orientations and visibly update.
            oRefs = osim.OrientationsReference(rot_table)
            ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordRefs, CONSTRAINT)
            ikSolver.setAccuracy(1e-4)
            state.setTime(0.0)
            if tibia_only != last_tibia_mode:
                # Lock flags persist in the state, so SWIG calls are only needed on a mode change.
                _apply_tibia_only_locks(coord_set, state, tibia_only, default_coord_locks)
                last_tibia_mode = tibia_only
            if live_frame <= 5 or live_frame % 1000 == 0:
                q_dbg = quats[0] if quats else []
                print(f"[IK] calling assemble frame={live_frame} t={t_imu:.3f}s sensors={active_sensors} q0={[round(v,3) for v in q_dbg]}")
            try:
                ikSolver.assemble(state)
            except Exception as e:
                print(f"[ERROR] OpenSim IK failed at frame={live_frame} t={t_imu:.3f}s")
                print(f"  Reason: {e}")
                print("  Check that orientation labels match model frames and quaternions are valid.")
                continue
            state.setTime(t_imu)

            live_frame += 1
            if live_frame % 25 == 0:
                display_joint = _load_display_joint(reload=True)

            joint_name = display_joint["joint"]
            joint_label = display_joint["label"]
            joint_index = display_joint["joint_index"]
            angle_deg = _read_coord_value(model, state, joint_name)

            if live_frame <= 5 or live_frame % 60 == 0:
                angle_text = f"{angle_deg:.2f}" if math.isfinite(angle_deg) else "nan"
                print(
                    f"[COORD {live_frame}] t={state.getTime():.3f} "
                    f"{joint_name}={angle_text} deg"
                )

            _send_angle_feedback(t_imu, joint_index, angle_deg)

            if math.isfinite(angle_deg):
                if last_coord_values is not None:
                    coord_delta = abs(angle_deg - last_coord_values)
                    if active_static and active_gyro_norm < STATIC_GYRO_EPS and coord_delta > STATIC_COORD_EPS_DEG:
                        print(
                            "FAIL: motion is not data-driven "
                            f"active_static=true active_gyro_norm={active_gyro_norm:.3f} "
                            f"active_change={active_change:.3f} coord_delta_deg={coord_delta:.3f}"
                        )
                last_coord_values = angle_deg

            _render_joint_display_hud(
                model, viz, state, hud_strategy, last_known=last_hud_angles
            )

            if live_frame <= 5 or live_frame % 1000 == 0:
                print(f"[VIZ] show frame={live_frame} t={state.getTime():.3f}s")
            model.getVisualizer().show(state)
            frame_times.append(time.perf_counter() - t_frame0)

            now_perf = time.time()
            if now_perf - t_last_perf_print >= 1.0 and frame_times:
                avg_frame = sum(frame_times) / len(frame_times) * 1000.0
                avg_convert = sum(convert_times) / len(convert_times) * 1000.0 if convert_times else 0.0
                warn = f"  <-- SLOW (>{target_frame_ms:.1f}ms)!" if avg_frame > target_frame_ms else ""
                print(f"[PERF] avg frame={avg_frame:.1f}ms  avg convert={avg_convert:.1f}ms  target={target_frame_ms:.1f}ms{warn}")
                knee_deg = _read_coord_value(model, state, "knee_angle_r")
                knee_text = f"{knee_deg:.2f}" if math.isfinite(knee_deg) else "nan"
                print(
                    f"[HUD-DIAG] t={state.getTime():.2f}s knee_angle_r={knee_text} deg "
                    f"filter={get_display_filter_joints()} sensors={active_sensors}"
                )
                frame_times.clear()
                convert_times.clear()
                t_last_perf_print = now_perf

            last_t = t_imu

    except KeyboardInterrupt:
        print("\n[Connect OpenSim] Stopped by user.")
    finally:
        _udp_running = False
        global _angle_feedback_sock
        if _angle_feedback_sock is not None:
            try:
                _angle_feedback_sock.close()
            except Exception:
                pass
            _angle_feedback_sock = None
        _joint_display_watcher_running = False


if __name__ == "__main__":
    if os.environ.get("OPENSIM_JOINT_DISPLAY_TEST") == "1":
        _write_test_joint_display_config(["knee_angle_r", "hip_flexion_r"], 1)
        time.sleep(0.25)
        _write_test_joint_display_config(["ankle_angle_r"], 2)
        print(
            "[JOINT-DISPLAY-TEST] Wrote seq=1 (knee_r, hip_r) then seq=2 (ankle_r). "
            "Observe [JOINT-DISPLAY] log within 200 ms of each write when run_live starts."
        )
    run_live()
