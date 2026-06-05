import sys
import os
import sys

# Register OpenSim DLL directories BEFORE any other imports.
# Python 3.8+ ignores PATH for extension-module DLL loading; add_dll_directory is mandatory.
if hasattr(os, "add_dll_directory"):
    for _dll_dir in [
        r"C:\OpenSim 4.5\bin",
        r"C:\OpenSim 4.5\sdk\lib",
        r"C:\OpenSim 4.5\sdk\Python\opensim",
    ]:
        if os.path.isdir(_dll_dir):
            try:
                os.add_dll_directory(_dll_dir)
            except Exception:
                pass

import subprocess
import socket
import struct
import time
import argparse
import math
import datetime
import json
import xml.etree.ElementTree as ET
import numpy as np
import imufusion

WORK_DIR          = r"C:\Users\KIN Student\Open-Sim--Bio-Mech"
OPENSIM_SDK_PYTHON = r"C:\OpenSim 4.5\sdk\Python"
OPENSIM_BIN_DIR   = r"C:\OpenSim 4.5\bin"
MODEL_FILE        = "Rajagopal2015_opensense_calibrated.osim"
TIBIA_ONLY_MODEL_FILE = "Rajagopal2015_tibia_only_locked.osim"
TIBIA_ONLY_UNLOCKED_COORDS = {"knee_angle_r", "knee_angle_r_beta"}

OPENSIM_CMD_CANDIDATES = [
    r"C:\OpenSim 4.5\bin\opensim-cmd.exe",
    r"C:\Program Files\OpenSim 4.5\bin\opensim-cmd.exe",
]

OPENSIM_GUI_CANDIDATES = [
    r"C:\OpenSim 4.5\bin\OpenSim64.exe",
    r"C:\Program Files\OpenSim 4.5\bin\OpenSim64.exe",
    r"C:\OpenSim 4.5\bin\opensim.exe",
    r"C:\Program Files\OpenSim 4.5\bin\opensim.exe",
]

EPHYS_SETUP_XML    = "ephys_imuIK_Setup.xml"
EPHYS_ORIENTATIONS = "ephys_live_orientations.sto"
EPHYS_OUTPUT       = "ephys_live_motion.sto"

SAMPLE_RATE = 100.0   # Hz — must match Open Ephys stream rate

UDP_HOST = "0.0.0.0"
UDP_PORT = 5000
AUTO_OPEN_GUI = True

FIRST_PACKET_TIMEOUT_S = 30.0  # seconds to wait for first packet before giving up

# ── Sensor-name mapping ───────────────────────────────────────────────────────
# Names must match PhysicalOffsetFrame names in Rajagopal2015_opensense_calibrated.osim.
# Available frames: torso_imu, pelvis_imu, femur_r_imu, tibia_r_imu,
#                   calcn_r_imu, femur_l_imu, tibia_l_imu, calcn_l_imu

# Distal → proximal: sensor 0 = tibia, 1 = thigh, 2 = hip, 3 = torso, ...
SENSOR_CHAIN_UP = [
    "tibia_r_imu",
    "femur_r_imu",
    "pelvis_imu",
    "torso_imu",
    "calcn_r_imu",
    "femur_l_imu",
    "tibia_l_imu",
    "calcn_l_imu",
    "humerus_r_imu",
    "radius_r_imu",
    "humerus_l_imu",
    "radius_l_imu",
    "head_imu",
]

SENSOR_NAMES = {
    8: list(SENSOR_CHAIN_UP),
}

# ESP32-S3 STEP node: single ICM20948, no on-board quaternion tail (set OPENSIM_ESP32_8CH=1).
if os.environ.get("OPENSIM_ESP32_8CH", "").strip().lower() in ("1", "true", "yes"):
    SENSOR_NAMES[1] = ["torso_imu"]

# Path written by the plugin just before launching this bridge.
SENSOR_MAP_FILE = os.path.join(WORK_DIR, "opensim_sensor_map.json")


def _load_sensor_map():
    """Read sensor_slots from opensim_sensor_map.json; return list or None."""
    if not os.path.isfile(SENSOR_MAP_FILE):
        return None
    try:
        with open(SENSOR_MAP_FILE, "r") as f:
            data = json.load(f)
        slots = data.get("sensor_slots", [])
        if slots and isinstance(slots, list):
            return [str(s) for s in slots if s]
    except Exception as e:
        print(f"[WARN] Could not read {SENSOR_MAP_FILE}: {e}")
    return None


def _sensor_names_for(n_imus):
    """Return the list of OpenSim frame names for n_imus sensors.

    Priority: hardcoded SENSOR_NAMES dict → opensim_sensor_map.json → SENSOR_CHAIN_UP fallback.
    """
    n = int(n_imus)
    if n in SENSOR_NAMES:
        return SENSOR_NAMES[n]
    mapped = _load_sensor_map()
    if mapped and 1 <= n <= len(mapped):
        return list(mapped[:n])
    if 1 <= n <= len(SENSOR_CHAIN_UP):
        return list(SENSOR_CHAIN_UP[:n])
    return [f"sensor_{i}_imu" for i in range(n)]


def run_opensim_ik(setup_xml):
    opensim_cmd = _find_opensim_cmd()
    if opensim_cmd is None:
        print("ERROR: opensim-cmd.exe was not found.")
        print("Checked:")
        for candidate in OPENSIM_CMD_CANDIDATES:
            print(f"  {candidate}")
        return False

    print(f"Running OpenSim IK: {setup_xml} ...")
    result = subprocess.run(
        [opensim_cmd, "run-tool", setup_xml],
        cwd=WORK_DIR, capture_output=True, text=True,
    )
    if result.returncode != 0:
        print("ERROR:", result.stderr.strip())
        return False
    print("IK complete.")
    return True


def _is_tibia_only_mode(sensor_names):
    return len(sensor_names) == 1 and sensor_names[0] == "tibia_r_imu"


def _write_tibia_only_model():
    src = os.path.join(WORK_DIR, MODEL_FILE)
    dst = os.path.join(WORK_DIR, TIBIA_ONLY_MODEL_FILE)

    tree = ET.parse(src)
    root = tree.getroot()

    for coord in root.iter("Coordinate"):
        locked = coord.find("locked")
        if locked is None:
            continue
        locked.text = "false" if coord.attrib.get("name") in TIBIA_ONLY_UNLOCKED_COORDS else "true"

    tree.write(dst, encoding="UTF-8", xml_declaration=True)
    return TIBIA_ONLY_MODEL_FILE


def _find_opensim_cmd():
    """Return the best available OpenSim command-line executable."""
    for candidate in OPENSIM_CMD_CANDIDATES:
        if os.path.isfile(candidate):
            return candidate
    return None


def _find_opensim_gui():
    """Return the best available OpenSim GUI executable."""
    for candidate in OPENSIM_GUI_CANDIDATES:
        if os.path.isfile(candidate):
            return candidate
    return None


# ── AHRS + STO helpers ────────────────────────────────────────────────────────

def ahrs_to_quaternions(accel_list, gyro_list):
    """Run Fusion AHRS on one sensor's accel/gyro lists -> list of [w,x,y,z]."""
    ahrs   = imufusion.Ahrs()
    offset = imufusion.Offset(int(SAMPLE_RATE))
    dt     = 1.0 / SAMPLE_RATE
    quats  = []
    for acc, gyr in zip(accel_list, gyro_list):
        gyr_cal = offset.update(np.array(gyr, dtype=np.float64))
        ahrs.update_no_magnetometer(
            np.array(gyr_cal, dtype=np.float64),
            np.array(acc,     dtype=np.float64),
            dt,
        )
        q = ahrs.quaternion
        quats.append([q.w, q.x, q.y, q.z])
    return quats


def _fmt_quat(q):
    qw, qx, qy, qz = q
    return f"{qw:.15g},{qx:.15g},{qy:.15g},{qz:.15g}"




def samples_to_sto(timestamps, quats_per_sensor, sensor_names, path):
    """Write an N-sensor quaternion STO file."""
    n = len(sensor_names)
    header_sensors = "\t".join(sensor_names)
    print(f"Writing STO with {n} IMU columns: {', '.join(sensor_names)}")
    with open(path, "w") as f:
        f.write("DataRate=100.000000\n")
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.1\n")
        f.write("endheader\n")
        f.write(f"time\t{header_sensors}\n")
        for row_idx, t in enumerate(timestamps):
            cols = "\t".join(_fmt_quat(quats_per_sensor[s][row_idx]) for s in range(n))
            f.write(f"{t:.6f}\t{cols}\n")
    print(f"Wrote {len(timestamps)} rows -> {path}")


# ── Post-collection pipeline ───────────────────────────────────────────────────

def _launch_opensim_gui(motion_path, run_timestamp=None, model_file=MODEL_FILE):
    """Launch OpenSim GUI.  run_timestamp is a datetime marking when IK started,
    used to warn if the motion file was not updated during this run."""
    model_path = os.path.join(WORK_DIR, model_file)
    gui_path   = _find_opensim_gui()

    # Resolve motion file mtime
    motion_mtime_dt  = None
    motion_mtime_str = "FILE NOT FOUND"
    if os.path.exists(motion_path):
        motion_mtime_dt  = datetime.datetime.fromtimestamp(os.path.getmtime(motion_path))
        motion_mtime_str = motion_mtime_dt.strftime("%Y-%m-%d %H:%M:%S")

    print("\n" + "=" * 62)
    print("Opening OpenSim with latest generated file:")
    print(f"  Model:  {model_path}")
    print(f"  Motion: {motion_path}")
    print(f"  Motion file modified at: {motion_mtime_str}")

    # Stale-file guard
    if run_timestamp is not None and motion_mtime_dt is not None:
        if motion_mtime_dt < run_timestamp:
            print()
            print("  WARNING: ephys_live_motion.sto was NOT updated during this run.")
            print(f"    File mtime : {motion_mtime_str}")
            print(f"    Run started: {run_timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
            print("  The file shown in OpenSim may be from a previous session.")
    print("=" * 62)

    if gui_path is None:
        print("  WARNING: OpenSim GUI executable not found. Open manually:")
        print(f"    File -> Open Model...  -> {model_path}")
        print(f"    File -> Load Motion... -> {motion_path}")
        return

    if not os.path.exists(model_path):
        print(f"  WARNING: Model file not found: {model_path}")
    if not os.path.exists(motion_path):
        print(f"  WARNING: Motion file not found: {motion_path}")

    try:
        subprocess.Popen(
            [gui_path, "--open", model_path, motion_path],
            cwd=WORK_DIR,
            close_fds=True,
        )
        print(f"  OpenSim GUI launched ({gui_path})")
        print("  If the motion timeline is empty, use:")
        print(f"    File -> Load Motion... -> {motion_path}")
    except FileNotFoundError:
        print(f"  WARNING: OpenSim GUI not found at {gui_path}")
    except Exception as e:
        print(f"  WARNING: Could not launch OpenSim GUI: {e}")
        print(f"    File -> Open Model...  -> {model_path}")
        print(f"    File -> Load Motion... -> {motion_path}")


def _process_and_run_ik(imu_accels, imu_gyros, sample_count, receive_duration_s=None, quat_stream=None):
    n_imus = len(quat_stream) if quat_stream is not None else len(imu_accels)
    if sample_count < 2 or n_imus == 0:
        print("[ERROR] No live IMU data received.")
        print("  Reason: LFP Viewer/Open Ephys is not sending packets, or wrong port is used.")
        print(f"  Expected UDP packets on {UDP_HOST}:{UDP_PORT}")
        return

    run_start    = datetime.datetime.now()
    sensor_names = _sensor_names_for(n_imus)
    dt           = 1.0 / SAMPLE_RATE
    timestamps   = [i * dt for i in range(sample_count)]
    sto_path     = os.path.join(WORK_DIR, EPHYS_ORIENTATIONS)
    motion_path  = os.path.join(WORK_DIR, EPHYS_OUTPUT)
    motion_dur_s = sample_count / SAMPLE_RATE

    print(f"\n[UPDATE] {n_imus} sensor(s): {', '.join(sensor_names)}  |  {sample_count} frames  |  {motion_dur_s:.1f}s")
    quats_per_sensor = []
    if quat_stream is not None and all(len(q) == sample_count for q in quat_stream):
        print("[QUAT] Using plugin quaternion stream (UDP v2); skipping AHRS.")
        quats_per_sensor = quat_stream
    else:
        for i in range(n_imus):
            print(f"[AHRS] Running fusion on sensor {i} ({sensor_names[i]})...")
            quats_per_sensor.append(ahrs_to_quaternions(imu_accels[i], imu_gyros[i]))

    model_file = _write_tibia_only_model() if _is_tibia_only_mode(sensor_names) else MODEL_FILE
    if _is_tibia_only_mode(sensor_names):
        print("[IK-MODE] tibia-only: locked all model coordinates except knee_angle_r and knee_angle_r_beta.")

    samples_to_sto(timestamps, quats_per_sensor, sensor_names, sto_path)
    ensure_ephys_xml(sensor_names, model_file=model_file)
    print("\n[IK] Running OpenSim IMU IK...")
    os.chdir(WORK_DIR)
    ik_ok = run_opensim_ik(EPHYS_SETUP_XML)
    if ik_ok:
        print("[SUCCESS] Motion file updated successfully.")
    else:
        print("[ERROR] OpenSim IK failed.")
        print("  Reason: Check that orientation labels match model frames, and that the model file exists.")
        return

    if AUTO_OPEN_GUI:
        _launch_opensim_gui(motion_path, run_timestamp=run_start, model_file=model_file)


# ── UDP packet reader ──────────────────────────────────────────────────────────

_logged_pkt_sizes: set = set()  # track sizes already announced to avoid log spam



def _parse_quat_v2_packet(data):
    n_floats = len(data) // 4
    if n_floats < 3:
        return None
    t0, ver, n_s = struct.unpack("<3f", data[:12])
    if not (1.99 < ver < 2.01):
        return None
    n_sensors = int(round(n_s))
    if n_sensors < 1:
        return None
    expected = 3 + n_sensors * 4
    if n_floats != expected:
        return None
    quats = struct.unpack(f"<{n_sensors * 4}f", data[12:12 + n_sensors * 16])
    return {"format": "quat_v2", "timestamp": t0, "n_imus": n_sensors, "quats": quats}

def _recv_packet(sock):
    """Receive one UDP packet.

    Returns dict:
      n_imus  : int   — number of complete IMUs (packet_floats // 6)
      accels  : [[ax,ay,az], ...] for each IMU
      gyros   : [[gx,gy,gz], ...] for each IMU
    or None on timeout / malformed packet.
    Supported packet sizes: 6, 12, 36, 48 floats (1, 2, 6, 8 IMUs).
    """
    try:
        data, _ = sock.recvfrom(4096)
    except socket.timeout:
        return None
    quat_pkt = _parse_quat_v2_packet(data)
    if quat_pkt is not None:
        key = ("quat_v2", quat_pkt["n_imus"])
        if key not in _logged_pkt_sizes:
            _logged_pkt_sizes.add(key)
            print(f"[UDP] Received quat v2 packet: {quat_pkt['n_imus']} sensor(s)")
        return quat_pkt

    n_floats = len(data) // 4
    n_imus   = n_floats // 6
    if n_imus < 1:
        return None
    if n_floats not in _logged_pkt_sizes:
        _logged_pkt_sizes.add(n_floats)
        print(f"[UDP] Received {n_floats}-float packet = {n_imus} IMUs  "
              f"(bytes={len(data)}, sensors: {', '.join(_sensor_names_for(n_imus))})")
    values = struct.unpack(f"{n_imus * 6}f", data[:n_imus * 6 * 4])
    accels, gyros = [], []
    for i in range(n_imus):
        base = i * 6
        accels.append([values[base],     values[base + 1], values[base + 2]])
        gyros.append( [values[base + 3], values[base + 4], values[base + 5]])
    return {"n_imus": n_imus, "accels": accels, "gyros": gyros}


def _log_sample(n, pkt):
    n_imus = pkt["n_imus"]
    if n <= 5 or n % 100 == 0:
        parts = []
        for i in range(min(n_imus, 3)):   # log first 3 IMUs max
            a = pkt["accels"][i]
            g = pkt["gyros"][i]
            parts.append(
                f"IMU{i} a=({a[0]:.2f},{a[1]:.2f},{a[2]:.2f}) "
                f"g=({g[0]:.2f},{g[1]:.2f},{g[2]:.2f})"
            )
        suffix = f"  +{n_imus - 3} more" if n_imus > 3 else ""
        print(f"  sample={n}  {'  '.join(parts)}{suffix}")


# ── Shared collection loop ─────────────────────────────────────────────────────

def _collect_loop(sock, deadline_fn, max_samples):
    """Collect packets until deadline_fn(last_packet_time) returns False.

    Returns (imu_accels, imu_gyros, sample_count) where
    imu_accels[i] / imu_gyros[i] are lists of [x,y,z] for IMU i.
    """
    imu_accels, imu_gyros = None, None
    quat_stream          = None
    sample_count         = 0
    announced            = False
    last_packet_time     = None
    detected_n_imus      = 0

    while deadline_fn(last_packet_time):
        if max_samples is not None and sample_count >= max_samples:
            print(f"Reached max-samples limit ({max_samples}).")
            break
        pkt = _recv_packet(sock)
        if pkt is None:
            continue

        if pkt.get("format") == "quat_v2":
            n = pkt["n_imus"]
            if not announced:
                sensor_names = _sensor_names_for(n)
                print(f"[CONNECTED] Receiving OpenSim UDP v2 quaternion packets.")
                print(f"[SENSORS] Detected {n} sensor(s): {', '.join(sensor_names)}")
                announced = True
                detected_n_imus = n
                quat_stream = [[] for _ in range(n)]
            quats = pkt["quats"]
            for i in range(min(n, detected_n_imus)):
                base = i * 4
                quat_stream[i].append(list(quats[base:base + 4]))
        else:
            n = pkt["n_imus"]
            if not announced:
                sensor_names = _sensor_names_for(n)
                print(f"[CONNECTED] Receiving live IMU packets.")
                print(f"[SENSORS] Detected {n} IMU sensors: {', '.join(sensor_names)}")
                for idx, name in enumerate(sensor_names):
                    print(f"[MAP] sensor{idx} → {name}")
                announced       = True
                detected_n_imus = n
                imu_accels      = [[] for _ in range(n)]
                imu_gyros       = [[] for _ in range(n)]

            for i in range(min(n, detected_n_imus)):
                imu_accels[i].append(pkt["accels"][i])
                imu_gyros[i].append(pkt["gyros"][i])

        sample_count    += 1
        last_packet_time = time.monotonic()
        _log_sample(sample_count, pkt)

    if imu_accels is None:
        imu_accels, imu_gyros = [], []

    return imu_accels, imu_gyros, sample_count, quat_stream


# ── UDP listener modes ─────────────────────────────────────────────────────────

def listen_udp(duration_s=3.0, max_samples=None):
    """Collect packets for a fixed number of seconds, then process."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    sock.settimeout(0.1)
    print(f"Listening on UDP {UDP_HOST}:{UDP_PORT} for {duration_s} seconds...")

    deadline = time.monotonic() + duration_s
    imu_accels, imu_gyros, n, quat_stream = _collect_loop(
        sock,
        deadline_fn=lambda _: time.monotonic() < deadline,
        max_samples=max_samples,
    )
    sock.close()
    print(f"\nCollected {n} samples.")
    _process_and_run_ik(imu_accels, imu_gyros, n, quat_stream=quat_stream)


def listen_udp_until_idle(idle_s=2.0):
    """Wait indefinitely for packets, stop when none arrive for idle_s seconds."""
    for _stale in (EPHYS_ORIENTATIONS, EPHYS_OUTPUT):
        _p = os.path.join(WORK_DIR, _stale)
        if os.path.exists(_p):
            os.remove(_p)
            print(f"[CLEANUP] Deleted stale file: {_stale}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    sock.settimeout(0.1)
    print(f"[START] OpenSim live bridge starting...")
    print(f"[WAITING] Waiting for LFP Viewer/Open Ephys IMU packets on {UDP_HOST}:{UDP_PORT}...")

    first_pkt_clock = [None]
    start_clock     = time.monotonic()
    timed_out       = [False]

    def not_idle(lpt):
        if lpt is None:
            if time.monotonic() - start_clock > FIRST_PACKET_TIMEOUT_S:
                timed_out[0] = True
                return False
            return True
        if first_pkt_clock[0] is None:
            first_pkt_clock[0] = time.monotonic()
        return time.monotonic() - lpt < idle_s

    imu_accels, imu_gyros, n, quat_stream = _collect_loop(sock, deadline_fn=not_idle, max_samples=None)
    stream_end = time.monotonic()
    sock.close()

    if timed_out[0]:
        print(f"\n[ERROR] No live IMU data received for {FIRST_PACKET_TIMEOUT_S:.0f} seconds.")
        print("  Reason: LFP Viewer/Open Ephys is not sending packets, or wrong port is used.")
        print(f"  Expected UDP packets on {UDP_HOST}:{UDP_PORT}")
        return

    print(f"\n[UPDATE] No packets for {idle_s:.0f} seconds — stream ended. Processing {n} frames...")

    receive_duration_s = None
    if first_pkt_clock[0] is not None:
        receive_duration_s = max(0.0, stream_end - first_pkt_clock[0] - idle_s)

    _process_and_run_ik(imu_accels, imu_gyros, n, receive_duration_s=receive_duration_s, quat_stream=quat_stream)


# ── Setup XML ──────────────────────────────────────────────────────────────────

def ensure_ephys_xml(sensor_names, model_file=MODEL_FILE):
    path = os.path.join(WORK_DIR, EPHYS_SETUP_XML)
    content = f"""<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
\t<IMUInverseKinematicsTool>
\t\t<model_file>{model_file}</model_file>
\t\t<results_directory>.</results_directory>
\t\t<time_range>0 999</time_range>
\t\t<output_motion_file>{EPHYS_OUTPUT}</output_motion_file>
\t\t<report_errors>false</report_errors>
\t\t<sensor_to_opensim_rotations>-1.5707963267948966 0 0</sensor_to_opensim_rotations>
\t\t<orientations_file>{EPHYS_ORIENTATIONS}</orientations_file>
\t</IMUInverseKinematicsTool>
</OpenSimDocument>"""
    with open(path, "w") as f:
        f.write(content)
    print(f"Created {EPHYS_SETUP_XML} (sensors: {', '.join(sensor_names)})")


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Open Ephys -> OpenSim UDP bridge (live)")
    parser.add_argument("--seconds", type=float, default=None,
                        help="Collect for a fixed number of seconds then process")
    parser.add_argument("--until-idle", type=float, default=None, metavar="IDLE_S",
                        help="Stop collecting when no packets arrive for IDLE_S seconds")
    parser.add_argument("--max-samples", type=int, default=None,
                        help="Stop after this many samples regardless of time")
    parser.add_argument("--no-open-gui", action="store_true",
                        help="Do not launch OpenSim GUI after ephys_live_motion.sto is created")
    args = parser.parse_args()

    AUTO_OPEN_GUI = not args.no_open_gui

    if args.until_idle is not None:
        listen_udp_until_idle(idle_s=args.until_idle)
    else:
        listen_udp(duration_s=args.seconds if args.seconds is not None else 3.0,
                   max_samples=args.max_samples)
