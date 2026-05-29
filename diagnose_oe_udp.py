"""
diagnose_oe_udp.py  -  Python 3.x  (no OpenSim, no imufusion needed)

Listens on UDP port 5000 and prints exactly what Open Ephys sends.
Run this INSTEAD of opensim_live_realtime.py to isolate the UDP layer.

Usage:
  1. Kill all python.exe (close all Python windows)
  2. py diagnose_oe_udp.py
  3. In Open Ephys: Fake IMU Stream ON, 8 IMUs, press Play
  4. Watch the output for 15 seconds then Ctrl+C
  5. Paste the output here so we can verify the data path

What this tells us:
  - Check 1: Are packets arriving at all?
  - Check 2: Is packet size correct (196 bytes for 8 IMUs)?
  - Check 3: Are gyro values non-zero after 2 seconds?
"""

import socket
import struct
import time

PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(("0.0.0.0", PORT))
except OSError as e:
    print(f"[FAIL] Cannot bind port {PORT}: {e}")
    print("  Another process holds port 5000.")
    print("  Fix in PowerShell:")
    print("    $p=(netstat -ano|Select-String ':5000 '|Select-Object -First 1|ForEach-Object{($_.ToString().Trim()-split'\\s+')[-1]});if($p){Stop-Process -Id $p -Force;\"Killed $p\"}else{'Free'}")
    raise SystemExit(1)

sock.settimeout(2.0)
print("=" * 60)
print(f"  diagnose_oe_udp.py  |  listening on port {PORT}")
print("=" * 60)
print("  Now press Play in Open Ephys (Fake IMU Stream, 8 IMUs)")
print("  Ctrl+C to stop after ~15 seconds")
print()

pkt_n = 0
t_start = None
last_log_t = 0.0

try:
    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            if pkt_n == 0:
                print("[WAIT] No packets yet — is Open Ephys Play active?")
            continue

        pkt_n += 1
        now = time.monotonic()
        if t_start is None:
            t_start = now
            print(f"[FIRST PACKET] from {addr}  size={len(data)} bytes  "
                  f"expected=196 bytes (8 IMUs)")

        elapsed = now - t_start

        n_floats_total = len(data) // 4
        n_imu_floats   = n_floats_total - 1          # subtract timestamp float
        n_imus         = n_imu_floats // 6

        if len(data) >= 4 * n_floats_total:
            all_f = struct.unpack(f"<{n_floats_total}f", data[:n_floats_total * 4])
        else:
            all_f = None

        size_ok  = "PASS" if len(data) == 196 else f"FAIL(got {len(data)})"
        n_imu_ok = "PASS" if n_imus == 8  else f"FAIL(got {n_imus})"

        # IMU 2 = femur_r: offset 1 + 2*6 = 13 (ax ay az gx gy gz)
        if all_f and len(all_f) >= 19:
            ts  = all_f[0]
            ax2 = all_f[13]; ay2 = all_f[14]; az2 = all_f[15]
            gx2 = all_f[16]; gy2 = all_f[17]; gz2 = all_f[18]
            gyro_nonzero = "PASS" if abs(gy2) > 0.5 else "zero"
        else:
            ts = ax2 = ay2 = az2 = gx2 = gy2 = gz2 = float("nan")
            gyro_nonzero = "??"

        # Print first 5 packets, then every 200
        if pkt_n <= 5 or pkt_n % 200 == 0 or (elapsed - last_log_t) >= 3.0:
            last_log_t = elapsed
            print(f"[PKT {pkt_n:5d}]  t_elapsed={elapsed:6.2f}s  "
                  f"ts={ts:6.3f}  "
                  f"size={len(data)}B({size_ok})  "
                  f"n_imus={n_imus}({n_imu_ok})")
            print(f"           IMU2(femur_r) accel=[{ax2:6.3f},{ay2:6.3f},{az2:6.3f}]g  "
                  f"gyro=[{gx2:7.2f},{gy2:7.2f},{gz2:7.2f}]deg/s  gyro_y={gyro_nonzero}")

except KeyboardInterrupt:
    elapsed = time.monotonic() - (t_start or time.monotonic())
    print(f"\n[STOP]  {pkt_n} packets in {elapsed:.1f} s  "
          f"({pkt_n/elapsed:.0f} pkt/s)" if elapsed > 0 else f"\n[STOP] {pkt_n} packets")
finally:
    sock.close()
