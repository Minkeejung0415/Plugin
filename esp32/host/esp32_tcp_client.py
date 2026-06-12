"""
TCP client for ESP32-S3 STEP nodes (Open Ephys Ephys-Socket compatible).

Extends the Red Pitaya client pattern used in STEP — same handshake and header,
with optional 8-channel parsing (ICM20948 + DIO + camera/verify).
"""
from __future__ import annotations

import asyncio
import logging
import os
import struct
from dataclasses import dataclass

logger = logging.getLogger(__name__)

HEADER = struct.Struct("<iiHiii")
HEADER_SIZE = HEADER.size


@dataclass
class NodeSample:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    dio: int
    camera: int
    qw: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0


def _q15_to_unit(v: int) -> float:
    return float(v) / 32767.0


def _scale_ch(v: int, env_key: str, default: float = 1.0) -> float:
    return float(v) * float(os.environ.get(env_key, str(default)))


async def run_stream(host: str | None = None, port: int | None = None) -> None:
    h = host or os.environ.get("ESP32_NODE_HOST", "192.168.4.1")
    p = port if port is not None else int(os.environ.get("ESP32_NODE_PORT", "5000"))
    num_ch = int(os.environ.get("ESP32_NUM_CHANNELS", "11"))

    reader, writer = await asyncio.open_connection(h, p)
    try:
        writer.write(b"REDPITAYA\n")
        await writer.drain()
        line = await reader.readline()
        logger.info("handshake: %s", line.decode(errors="replace").strip())

        writer.write(b"START\n")
        await writer.drain()

        buf = bytearray()
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
            if n_ch >= 6:
                s = NodeSample(
                    ax=_scale_ch(int(s16[0, 0]), "ICM_ACCEL_SCALE"),
                    ay=_scale_ch(int(s16[1, 0]), "ICM_ACCEL_SCALE"),
                    az=_scale_ch(int(s16[2, 0]), "ICM_ACCEL_SCALE"),
                    gx=_scale_ch(int(s16[3, 0]), "ICM_GYRO_SCALE"),
                    gy=_scale_ch(int(s16[4, 0]), "ICM_GYRO_SCALE"),
                    gz=_scale_ch(int(s16[5, 0]), "ICM_GYRO_SCALE"),
                    dio=int(s16[6, 0]) if n_ch > 6 else 0,
                    camera=0,
                    qw=_q15_to_unit(int(s16[7, 0])) if n_ch > 7 else 0.0,
                    qx=_q15_to_unit(int(s16[8, 0])) if n_ch > 8 else 0.0,
                    qy=_q15_to_unit(int(s16[9, 0])) if n_ch > 9 else 0.0,
                    qz=_q15_to_unit(int(s16[10, 0])) if n_ch > 10 else 0.0,
                )
                logger.debug("sample %s", s)
    finally:
        writer.close()
        await writer.wait_closed()


if __name__ == "__main__":
    import argparse

    logging.basicConfig(level=logging.INFO)
    ap = argparse.ArgumentParser(
        description="Test ESP32 STEP node TCP (REDPITAYA/START + binary stream)"
    )
    ap.add_argument(
        "--host",
        default=os.environ.get("ESP32_NODE_HOST", "192.168.4.1"),
        help="Node IP from Serial (WiFi OK IP=… or Soft AP 192.168.4.1)",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("ESP32_NODE_PORT", "5000")),
    )
    args = ap.parse_args()
    try:
        asyncio.run(run_stream(host=args.host, port=args.port))
    except (ConnectionRefusedError, OSError) as e:
        logging.error(
            "TCP connect failed to %s:%s — ping IP; check firewall; "
            "Serial must show 'TCP listen :5000' and same LAN (not client isolation). %s",
            args.host,
            args.port,
            e,
        )
        raise SystemExit(1) from e
