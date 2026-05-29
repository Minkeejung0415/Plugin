"""
udp_receive_test.py — Python 3.8, no OpenSim deps
Pure UDP receive test on port 5000.
Run this, then run fake_imu_sender_test.py in a second terminal.
"""
import socket
import struct

UDP_PORT = 5000

print(f"[TEST] Binding to 0.0.0.0:{UDP_PORT} ...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(("0.0.0.0", UDP_PORT))
    print(f"[TEST] Bind OK. Waiting for packets (Ctrl+C to stop) ...")
except OSError as e:
    print(f"[ERROR] Bind FAILED: {e}")
    print("[ERROR] Another process is holding port 5000.")
    raise SystemExit(1)

sock.settimeout(10.0)
pkt_count = 0
try:
    while pkt_count < 10:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            print("[TIMEOUT] No packet in 10s — sender not running or blocked.")
            break
        pkt_count += 1
        n_floats = len(data) // 4
        n_imus   = n_floats // 6
        values   = struct.unpack(f"<{n_floats}f", data[:n_floats * 4])
        ax, ay, az = values[0], values[1], values[2]
        print(f"[PKT {pkt_count:3d}] {len(data)} bytes | {n_imus} IMUs | "
              f"IMU0 accel = [{ax:.4f}, {ay:.4f}, {az:.4f}] g  from {addr}")
finally:
    sock.close()

if pkt_count > 0:
    print(f"\n[PASS] Received {pkt_count} packets — UDP transport is working.")
else:
    print("\n[FAIL] Zero packets received.")
