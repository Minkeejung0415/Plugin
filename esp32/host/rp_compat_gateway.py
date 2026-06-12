#!/usr/bin/env python3
"""
Red Pitaya–compatible gateway for ESP32-S3 + USB (no Plugin C++ rebuild).

Satisfies the three Acquisition Board requirements without editing the Plugin:
  1. Host — add to Windows hosts: 127.0.0.1 rp-f0f85a.local  (see plugin-patches/hosts.txt)
  2. Data on UDP 55001 — this process sends each sample as a UDP datagram
  3. No SENSORS stall — replies STARTED + SENSORS:0,ICM20948 after START

Wire: ESP32 USB (binary) → this script → Plugin (TCP :5000 + UDP :55001).

Usage:
  pip install pyserial
  python host/rp_compat_gateway.py COM5

Plugin: unchanged binary; point at rp-f0f85a.local (resolves to 127.0.0.1).
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import socket
import struct
import sys
import threading
import time
from collections import deque

from serial_tcp_bridge import (
    HEADER,
    HEADER_SIZE,
    SerialFrameSource,
)

logger = logging.getLogger(__name__)

DEFAULT_NUM_CHANNELS = int(os.environ.get("ESP32_NUM_CHANNELS", "11"))
UDP_PORT = int(os.environ.get("RP_UDP_PORT", "55001"))
TCP_PORT = int(os.environ.get("RP_TCP_PORT", "5000"))
BIND = os.environ.get("RP_GATEWAY_BIND", "0.0.0.0")
UDP_TARGET = os.environ.get("RP_UDP_TARGET", "127.0.0.1")
SENSORS_LINE = os.environ.get(
    "RP_SENSORS_LINE",
    "SENSORS:0,ICM20948\n",
)


async def read_line(reader: asyncio.StreamReader) -> str:
    line = await reader.readline()
    return line.decode("utf-8", errors="replace").strip()


async def handle_tcp_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    source: SerialFrameSource,
    udp_sock: socket.socket,
    num_ch: int,
) -> None:
    peer = writer.get_extra_info("peername")
    logger.info("TCP client %s", peer)
    streaming = False
    try:
        while True:
            line = await asyncio.wait_for(read_line(reader), timeout=120.0)
            if not line:
                break
            upper = line.upper()
            if upper.startswith("REDPITAYA"):
                source.write_line("REDPITAYA\n")
                writer.write(f"OK CHANNELS:{num_ch}\n".encode())
                await writer.drain()
                writer.write(
                    f"{num_ch} channels; sample_rate=100; fusion=madgwick; node=esp32s3_gateway\n".encode()
                )
                await writer.drain()
            elif upper.startswith("START"):
                source.write_line("START\n")
                source.drain()
                writer.write(b"STARTED\n")
                await writer.drain()
                writer.write(SENSORS_LINE.encode())
                await writer.drain()
                streaming = True
                logger.info("STARTED + SENSORS — streaming on UDP %d", UDP_PORT)
            elif upper.startswith("FILTER"):
                source.write_line(line + "\n")
            elif not streaming:
                logger.debug("cmd: %s", line)
    except asyncio.TimeoutError:
        logger.warning("TCP idle timeout")
    except ConnectionResetError:
        pass
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass

    if not streaming:
        return

    loop = asyncio.get_event_loop()
    while not reader.at_eof():
        frame = await loop.run_in_executor(None, source.get_frame, 1.0)
        if frame is None:
            continue
        try:
            udp_sock.sendto(frame, (UDP_TARGET, UDP_PORT))
        except OSError as e:
            logger.warning("UDP send failed: %s", e)


async def run_gateway(
    source: SerialFrameSource,
    num_ch: int,
    bind: str,
    tcp_port: int,
) -> None:
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server = await asyncio.start_server(
        lambda r, w: handle_tcp_client(r, w, source, udp_sock, num_ch),
        bind,
        tcp_port,
    )
    addrs = ", ".join(str(s.getsockname()) for s in server.sockets or [])
    logger.info("TCP %s (handshake)", addrs)
    logger.info("UDP samples → %s:%d", UDP_TARGET, UDP_PORT)
    logger.info("Hosts file: 127.0.0.1 rp-f0f85a.local (see plugin-patches/hosts.txt)")

    async with server:
        await server.serve_forever()


def main() -> None:
    p = argparse.ArgumentParser(description="ESP32 USB → RP-style TCP+UDP for Open Ephys Plugin")
    p.add_argument("port", nargs="?", default=os.environ.get("SERIAL_PORT"))
    p.add_argument("--baud", type=int, default=int(os.environ.get("SERIAL_BAUD", "115200")))
    p.add_argument("--bind", default=BIND)
    p.add_argument("--tcp-port", type=int, default=TCP_PORT)
    p.add_argument("--csv", action="store_true")
    args = p.parse_args()

    if not args.port:
        print("Usage: python host/rp_compat_gateway.py COM5", file=sys.stderr)
        sys.exit(1)

    num_ch = int(os.environ.get("ESP32_NUM_CHANNELS", str(DEFAULT_NUM_CHANNELS)))
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    source = SerialFrameSource(args.port, args.baud, csv_mode=args.csv, num_ch=num_ch)
    source.start()
    try:
        asyncio.run(run_gateway(source, num_ch, args.bind, args.tcp_port))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        source.stop()


if __name__ == "__main__":
    main()
