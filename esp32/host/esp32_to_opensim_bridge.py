#!/usr/bin/env python3
"""
ESP32-S3 STEP → OpenSim UDP bridge (quaternion stream).

Reads Open Ephys–compatible TCP from the node (or localhost serial bridge),
extracts quaternion channels 7–10 (0-based), and sends 4 floats to OpenSim.

Channel map (firmware v1.4+):
  ch0–2  accel (raw or gravity-removed when FILTER 1)
  ch3–5  gyro raw
  ch6    DIO
  ch7–10 qw, qx, qy, qz (int16 / 32767)

Usage:
  python host/esp32_to_opensim_bridge.py
  set ESP32_NODE_HOST=127.0.0.1  (USB: run serial_tcp_bridge.py first)
  set OPENSIM_UDP_HOST=127.0.0.1 OPENSIM_UDP_PORT=9876
"""
from __future__ import annotations

import asyncio
import logging
import os
import socket
import struct

logger = logging.getLogger(__name__)

HEADER = struct.Struct("<iiHiii")
HEADER_SIZE = HEADER.size
QUAT_SCALE = 1.0 / 32767.0


def send_quat_udp(sock: socket.socket, host: str, port: int, q: tuple[float, float, float, float]) -> None:
    payload = struct.pack("<4f", *q)
    sock.sendto(payload, (host, port))


async def run() -> None:
    host = os.environ.get("ESP32_NODE_HOST", "127.0.0.1")
    port = int(os.environ.get("ESP32_NODE_PORT", "5000"))
    udp_host = os.environ.get("OPENSIM_UDP_HOST", "127.0.0.1")
    udp_port = int(os.environ.get("OPENSIM_UDP_PORT", "9876"))
    expect_ch = int(os.environ.get("ESP32_NUM_CHANNELS", "11"))

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    reader, writer = await asyncio.open_connection(host, port)
    try:
        writer.write(b"REDPITAYA\n")
        await writer.drain()
        line = await reader.readline()
        logger.info("handshake: %s", line.decode(errors="replace").strip())
        if expect_ch >= 11:
            line2 = await reader.readline()
            if line2:
                logger.info("handshake: %s", line2.decode(errors="replace").strip())

        writer.write(b"START\n")
        await writer.drain()
        start_ack = await reader.readline()
        if start_ack:
            logger.info("start: %s", start_ack.decode(errors="replace").strip())

        buf = bytearray()
        n_sent = 0
        while True:
            while len(buf) < HEADER_SIZE:
                buf.extend(await reader.read(4096))
            hdr = HEADER.unpack_from(buf, 0)
            _off, num_bytes, _bd, elem, n_ch, n_per = hdr
            if elem != 2:
                raise ValueError("expected int16 payload")
            total = HEADER_SIZE + num_bytes
            while len(buf) < total:
                buf.extend(await reader.read(4096))
            payload = memoryview(buf)[HEADER_SIZE:total]
            del buf[:total]

            import numpy as np

            s16 = np.frombuffer(payload, dtype="<i2").reshape(n_ch, n_per, order="C")
            if n_ch < 11:
                logger.warning("expected >=11 channels, got %d", n_ch)
                continue
            qw = float(s16[7, 0]) * QUAT_SCALE
            qx = float(s16[8, 0]) * QUAT_SCALE
            qy = float(s16[9, 0]) * QUAT_SCALE
            qz = float(s16[10, 0]) * QUAT_SCALE
            send_quat_udp(udp, udp_host, udp_port, (qw, qx, qy, qz))
            n_sent += 1
            if n_sent % 500 == 0:
                logger.info("sent %d quaternions → %s:%d", n_sent, udp_host, udp_port)
    finally:
        writer.close()
        await writer.wait_closed()
        udp.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    asyncio.run(run())
