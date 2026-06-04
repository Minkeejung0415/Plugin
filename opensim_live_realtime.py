"""
opensim_live_realtime.py  -  Python 3.8 ONLY
Real-time OpenSim IK with Simbody visualizer.
Receives UDP from Open Ephys: v2 quaternion packets (t, version=2, N, N×qw,qx,qy,qz) or legacy acc/gyro IMU packets
on port 5000, same format as the Open Ephys bridge sends.
Runs imufusion AHRS per sensor -> quaternions -> OpenSim orientation IK
-> Simbody visualizer update.
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

UDP_PACKET_VERSION_QUAT = 2.0
_sensor_map_cache = None

try:
    osim.Logger.setLevelString("Warn")
except Exception:
    pass

MODEL_PATH = r"C:\Users\KIN Student\Open-Sim--Bio-Mech\Rajagopal2015_opensense_calibrated.osim"
WORK_DIR = r"C:\Users\KIN Student\Open-Sim--Bio-Mech"
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

SENSORS = [
    "torso_imu",
    "pelvis_imu",
    "femur_r_imu",
    "tibia_r_imu",
    "calcn_r_imu",
    "femur_l_imu",
    "tibia_l_imu",
    "calcn_l_imu",
]
N_SENSORS = len(SENSORS)

# Distal → proximal: RP sensor index 0 = shank/tibia, 1 = thigh, 2 = hip, 3 = trunk, ...
SENSOR_CHAIN_UP = [
    "tibia_r_imu",
    "femur_r_imu",
    "pelvis_imu",
    "torso_imu",
    "calcn_r_imu",
    "femur_l_imu",
    "tibia_l_imu",
    "calcn_l_imu",
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
    return [_quat_mul(_Q_OPENSIM_FRAME, q) for q in _NEUTRAL_QUATS_8IMU]


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


_Q_OPENSIM_FRAME = _qx(-math.pi / 2.0)



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


def _parse_quat_v2_packet(data):
    n_floats = len(data) // 4
    if n_floats < 3:
        return None
    t0, ver, n_s = struct.unpack("<3f", data[:12])
    if not (1.99 < ver < 2.01):
        return None
    n_sensors = int(round(n_s))
    if n_sensors < 1 or n_sensors > N_SENSORS:
        return None
    expected = 3 + n_sensors * 4
    if n_floats != expected:
        return None
    quats = struct.unpack(f"<{n_sensors * 4}f", data[12:12 + n_sensors * 16])
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
    ahrs_list = [imufusion.Ahrs() for _ in range(N_SENSORS)]
    offset_list = [imufusion.Offset(int(SAMPLE_RATE)) for _ in range(N_SENSORS)]

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
            packet_timestamp = struct.unpack("<f", data[:4])[0]
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
            offset_list = [imufusion.Offset(int(SAMPLE_RATE)) for _ in range(N_SENSORS)]
            with _frame_lock:
                _frame_queue.clear()
            dt = 1.0 / SAMPLE_RATE
            last_source_key = source_key
            last_expanded_values = None
            neutral_static_since = None
            neutral_warned = False
            _connected_announced = False
            if "source/slot changed" in reason:
                print(f"[SESSION] source changed, reset AHRS id={session_id} reason={reason}")
            print(f"[SESSION] new play session detected id={session_id} reason={reason}")
        else:
            dt = max(1.0 / SAMPLE_RATE, packet_timestamp - last_packet_timestamp)
            last_packet_timestamp = packet_timestamp

        raw_values = struct.unpack(
            f"<{packet_imus * 6}f",
            data[imu_start:imu_start + packet_imus * 6 * 4],
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

        active_quats   = []
        active_sensors = []

        for i in range(N_SENSORS):
            base = i * 6
            ax, ay, az = values[base], values[base + 1], values[base + 2]
            gx, gy, gz = values[base + 3], values[base + 4], values[base + 5]

            gyr_cal = offset_list[i].update(np.array([gx, gy, gz], dtype=np.float64))
            ahrs_list[i].update_no_magnetometer(
                gyr_cal,
                np.array([ax, ay, az], dtype=np.float64),
                dt,
            )

            if i not in slot_map:
                continue  # skip slots with no real sensor — do not feed neutral filler to IK

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
    for _stale in ("ephys_live_orientations.sto", "ephys_live_motion.sto", "_neutral_frame.sto"):
        _p = os.path.join(WORK_DIR, _stale)
        if os.path.exists(_p):
            os.remove(_p)
            print(f"[CLEANUP] Deleted stale file: {_stale}")
    print("[Connect OpenSim] Loading model ...")
    model = osim.Model(MODEL_PATH)
    model.setUseVisualizer(True)

    neutral_sto = os.path.join(WORK_DIR, "_neutral_frame.sto")
    _write_quat_sto(neutral_sto, 0.0, [_quat_mul(_Q_OPENSIM_FRAME, q) for q in _NEUTRAL_QUATS_8IMU])
    neutral_qt = osim.TimeSeriesTableQuaternion(neutral_sto)
    neutral_rt = osim.OpenSenseUtilities.convertQuaternionsToRotations(neutral_qt)

    mRefs = osim.MarkersReference()
    coordRefs = osim.SimTKArrayCoordinateReference()

    print("[Connect OpenSim] Initialising IK solver ...")
    state = model.initSystem()
    oRefs = osim.OrientationsReference(neutral_rt)
    ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordRefs, CONSTRAINT)
    ikSolver.setAccuracy(1e-4)
    default_coord_locks = _capture_coordinate_locks(model.getCoordinateSet(), state)

    viz = model.updVisualizer().updSimbodyVisualizer()
    try:
        viz.setWindowTitle("Connect OpenSim - RedPitaya 8-IMU")
    except Exception:
        pass
    viz.setShowSimTime(True)

    print("[Connect OpenSim] Assembling neutral pose ...")
    ikSolver.assemble(state)
    model.getVisualizer().show(state)
    print("[Connect OpenSim] Neutral pose assembled. Waiting for IMU stream ...")

    global _udp_running
    t_recv = threading.Thread(target=_udp_ahrs_thread, daemon=True)
    t_recv.start()

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
        time.sleep(0.01)

    print("[Connect OpenSim] Streaming started. Close the Simbody window to stop.")

    last_t = -1.0
    last_session_id = None
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
    coord_set = model.getCoordinateSet()

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
                time.sleep(0.005)
                continue

            last_frame_wall_time = packet_wall_time
            if session_id != last_session_id:
                print(f"[SESSION] renderer reset id={session_id}")
                last_session_id = session_id
                last_t = -1.0
                t_last_solve = 0.0
                last_coord_values = None
                mode = "tibia-only knee lock" if _is_tibia_only_mode(active_sensors) else "full active-sensor IK"
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
            if _is_tibia_only_mode(active_sensors):
                ik_quats = quats
                ik_sensors = active_sensors
            else:
                ik_quats = quats if active_sensors == SENSORS else _merge_live_quats_with_neutral(quats, active_sensors)
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
            _apply_tibia_only_locks(coord_set, state, _is_tibia_only_mode(active_sensors), default_coord_locks)
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
            current_coord_values = None
            if live_frame <= 5 or live_frame % 60 == 0:
                try:
                    hip_r = np.degrees(coord_set.get("hip_flexion_r").getValue(state))
                    knee_r = np.degrees(coord_set.get("knee_angle_r").getValue(state))
                    pelvis_tilt = np.degrees(coord_set.get("pelvis_tilt").getValue(state))
                    pelvis_list = np.degrees(coord_set.get("pelvis_list").getValue(state))
                    pelvis_rotation = np.degrees(coord_set.get("pelvis_rotation").getValue(state))
                    current_coord_values = [hip_r, knee_r, pelvis_tilt, pelvis_list, pelvis_rotation]
                    print(
                        f"[COORD {live_frame}] t={state.getTime():.3f} "
                        f"hip_flexion_r={hip_r:.2f} deg "
                        f"knee_angle_r={knee_r:.2f} deg "
                        f"pelvis_tilt={pelvis_tilt:.2f} deg "
                        f"pelvis_list={pelvis_list:.2f} deg "
                        f"pelvis_rotation={pelvis_rotation:.2f} deg"
                    )
                except Exception as e:
                    print(f"[COORD] read error: {e}")

            if current_coord_values is not None:
                if last_coord_values is not None:
                    coord_delta = max(abs(a - b) for a, b in zip(current_coord_values, last_coord_values))
                    if active_static and active_gyro_norm < STATIC_GYRO_EPS and coord_delta > STATIC_COORD_EPS_DEG:
                        print(
                            "FAIL: motion is not data-driven "
                            f"active_static=true active_gyro_norm={active_gyro_norm:.3f} "
                            f"active_change={active_change:.3f} coord_delta_deg={coord_delta:.3f}"
                        )
                last_coord_values = current_coord_values

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
                frame_times.clear()
                convert_times.clear()
                t_last_perf_print = now_perf

            last_t = t_imu

    except KeyboardInterrupt:
        print("\n[Connect OpenSim] Stopped by user.")
    finally:
        _udp_running = False


if __name__ == "__main__":
    run_live()
