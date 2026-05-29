import socket
import struct
import time

HOST = "127.0.0.1"
PORT = 5000
N_PACKETS = 5

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Sending {N_PACKETS} test packets to {HOST}:{PORT}")

for i in range(N_PACKETS):
    # Fake accel (ax, ay, az) and gyro (gx, gy, gz) as 6 floats
    ax, ay, az = 0.1 * i, 0.2 * i, 9.8
    gx, gy, gz = 0.01 * i, -0.01 * i, 0.0

    data = struct.pack("6f", ax, ay, az, gx, gy, gz)
    sock.sendto(data, (HOST, PORT))
    print(f"  Sent packet {i+1}: ax={ax:.2f} ay={ay:.2f} az={az:.2f}")
    time.sleep(0.1)

print("Done.")
