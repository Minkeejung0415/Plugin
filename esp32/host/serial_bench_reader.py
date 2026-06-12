#!/usr/bin/env python3
"""
Read STEP node samples from USB Serial (bench mode).

Sketch settings:
  ENABLE_TCP false
  ENABLE_SERIAL_BENCH true
  SERIAL_OUTPUT_BINARY false  -> CSV (default)
  SERIAL_OUTPUT_BINARY true   -> Open Ephys 22-byte header + int16 x8

Windows example:
  pip install pyserial
  python host/serial_bench_reader.py COM5
  set SERIAL_PORT=COM5 && python host/serial_bench_reader.py
"""
from __future__ import annotations

import argparse
import os
import struct
import sys

HEADER = struct.Struct("<iiHiii")
HEADER_SIZE = HEADER.size
NUM_CHANNELS = 8


def open_port(port: str, baud: int):
    try:
        import serial
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    return serial.Serial(port, baud, timeout=1)


def read_csv(ser, limit: int | None) -> None:
    n = 0
    print("seq,ax,ay,az,gx,gy,gz,dio,cam")
    while limit is None or n < limit:
        line = ser.readline()
        if not line:
            continue
        text = line.decode("utf-8", errors="replace").strip()
        if not text or text.startswith("STEP") or text.startswith("ICM") or text.startswith("Wi-"):
            print(f"# {text}", file=sys.stderr)
            continue
        if text.startswith("Format:") or text.startswith("Serial") or text.startswith("ESP-"):
            print(f"# {text}", file=sys.stderr)
            continue
        print(text)
        n += 1


def read_binary(ser, limit: int | None) -> None:
    buf = bytearray()
    n = 0
    while limit is None or n < limit:
        buf.extend(ser.read(4096))
        while len(buf) >= HEADER_SIZE:
            hdr = HEADER.unpack_from(buf, 0)
            _off, num_bytes, _bd, elem, n_ch, n_per = hdr
            if elem != 2:
                del buf[0]
                continue
            total = HEADER_SIZE + num_bytes
            if len(buf) < total:
                break
            payload = buf[HEADER_SIZE:total]
            del buf[:total]
            samples = struct.unpack("<" + "h" * (n_ch * n_per), payload)
            print(f"frame={n} ch={samples}")
            n += 1


def main() -> None:
    p = argparse.ArgumentParser(description="Read STEP ESP32-S3 serial bench stream")
    p.add_argument("port", nargs="?", default=os.environ.get("SERIAL_PORT"), help="COM5 on Windows")
    p.add_argument("--baud", type=int, default=int(os.environ.get("SERIAL_BAUD", "115200")))
    p.add_argument("--binary", action="store_true", help="Parse Open Ephys binary (SERIAL_OUTPUT_BINARY)")
    p.add_argument("--limit", type=int, default=100, help="Samples to print (0 = unlimited)")
    args = p.parse_args()
    if not args.port:
        print("Usage: python host/serial_bench_reader.py COM5", file=sys.stderr)
        sys.exit(1)

    limit = None if args.limit == 0 else args.limit
    with open_port(args.port, args.baud) as ser:
        if args.binary:
            read_binary(ser, limit)
        else:
            read_csv(ser, limit)


if __name__ == "__main__":
    main()
