"""
rec_download.py — Pull the last recording from the ESP32 SD card over USB serial.

Protocol flow:
  1. REC SESSION          → get metadata (file_size, checksum, sample_count)
  2. REC GET offset=N ... → binary SDRF frames containing raw SD file chunks
  3. REC COMPLETE         → acknowledge done

Output: a CSV file with columns:
  seq, time_us, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz, dio

Usage:
  python rec_download.py --port COM3 [--baud 2000000] [--out recording.csv]
"""

import argparse
import struct
import sys
import time
import serial

# ── SD file layout (matches step_node.ino, packed) ───────────────────────────

SD_LOG_MAGIC   = 0x31505453  # "STP1" little-endian
SDLOG_HEADER_FMT = "<IHHHHq"   # magic, version, record_size, sample_hz, channel_count, start_time_us
SDLOG_HEADER_SIZE = struct.calcsize(SDLOG_HEADER_FMT)  # 20 bytes

# SdLogRecord: seq(u32) + time_us(i64) + ch[N](i16 each)
# N is read from the header; we build the struct format dynamically.

# ── SDRF transfer frame (matches SdrfHeader, 64 bytes packed) ────────────────

SDRF_MAGIC      = b"SDRF"
SDRF_HEADER_FMT = "<4sBBH16sIQIQIIII"
SDRF_HEADER_SIZE = struct.calcsize(SDRF_HEADER_FMT)  # must be 64
assert SDRF_HEADER_SIZE == 64, f"SDRF header size mismatch: {SDRF_HEADER_SIZE}"

SDRF_TYPE_DATA = 0x01
SDRF_TYPE_EOF  = 0x02

CHUNK_SIZE = 1024  # must match REC_MAX_CHUNK in firmware

# ── CRC32 (same polynomial as firmware recCrc32Update) ───────────────────────

def _build_crc32_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if (c & 1) else c >> 1
        table.append(c)
    return table

_CRC32_TABLE = _build_crc32_table()

def crc32(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc = _CRC32_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc & 0xFFFFFFFF


# ── Serial helpers ────────────────────────────────────────────────────────────

def send(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd + "\n").encode())
    ser.flush()


def readline_timeout(ser: serial.Serial, timeout_s: float = 5.0) -> str:
    deadline = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < deadline:
        ch = ser.read(1)
        if ch:
            buf += ch
            if ch == b"\n":
                return buf.decode(errors="replace").strip()
    raise TimeoutError(f"No line received within {timeout_s}s (got: {buf!r})")


def parse_kv(line: str) -> dict:
    """Parse 'KEY val1=a val2=b ...' into {'val1':'a', 'val2':'b'}."""
    parts = line.split()
    out = {}
    for p in parts[1:]:
        if "=" in p:
            k, _, v = p.partition("=")
            out[k] = v
    return out


# ── REC SESSION ───────────────────────────────────────────────────────────────

def rec_session(ser: serial.Serial) -> dict:
    send(ser, "REC SESSION")
    for _ in range(10):
        line = readline_timeout(ser, timeout_s=5.0)
        if line.startswith("REC SESSION_OK"):
            kv = parse_kv(line)
            return {
                "session_id":    kv.get("session_id", ""),
                "file_size":     int(kv.get("file_size", 0)),
                "file_checksum": int(kv.get("file_checksum", "0"), 16),
                "sample_count":  int(kv.get("sample_count", 0)),
            }
        if line.startswith("REC ERR"):
            raise RuntimeError(f"REC SESSION failed: {line}")
    raise RuntimeError("REC SESSION: no SESSION_OK received")


# ── REC GET (single chunk) ────────────────────────────────────────────────────

def read_exact(ser: serial.Serial, n: int, timeout_s: float = 10.0) -> bytes:
    buf = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(buf) < n:
        if time.monotonic() > deadline:
            raise TimeoutError(f"Timeout reading {n} bytes (got {len(buf)})")
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def rec_get_chunk(ser: serial.Serial, offset: int, chunk_index: int) -> tuple[bytes, bool]:
    """
    Returns (payload_bytes, is_eof).
    Raises on error.
    """
    send(ser, f"REC GET offset={offset} length={CHUNK_SIZE} chunk_index={chunk_index}")

    # Read 64-byte SDRF header
    raw_hdr = read_exact(ser, SDRF_HEADER_SIZE, timeout_s=10.0)
    (magic, fver, ftype, hlen,
     session_id_raw,
     cidx, byte_offset, payload_len, total_size,
     hdr_crc, payload_crc, flags, _reserved) = struct.unpack(SDRF_HEADER_FMT, raw_hdr)

    if magic != SDRF_MAGIC:
        raise ValueError(f"Bad SDRF magic: {magic!r}")

    # Verify header CRC (CRC was computed with header_crc32 field zeroed)
    hdr_for_crc = bytearray(raw_hdr)
    # header_crc32 is at offset 44 (4+1+1+2+16+4+8+4+8 = 44)
    hdr_for_crc[44:48] = b"\x00\x00\x00\x00"
    computed_hdr_crc = crc32(bytes(hdr_for_crc))
    if computed_hdr_crc != hdr_crc:
        raise ValueError(f"SDRF header CRC mismatch: got {hdr_crc:#010x}, computed {computed_hdr_crc:#010x}")

    payload = b""
    if payload_len > 0:
        payload = read_exact(ser, payload_len, timeout_s=10.0)
        computed_pay_crc = crc32(payload)
        if computed_pay_crc != payload_crc:
            raise ValueError(f"SDRF payload CRC mismatch chunk {chunk_index}")

    is_eof = (ftype == SDRF_TYPE_EOF)
    return payload, is_eof


# ── Download entire file ──────────────────────────────────────────────────────

def download_file(ser: serial.Serial, file_size: int) -> bytes:
    buf = bytearray()
    chunk_index = 0
    while len(buf) < file_size:
        offset = len(buf)
        remaining = file_size - offset
        print(f"\r  {offset}/{file_size} bytes ({100*offset//file_size}%)", end="", flush=True)
        payload, is_eof = rec_get_chunk(ser, offset, chunk_index)
        buf.extend(payload)
        chunk_index += 1
        if is_eof:
            # Read the EOF frame (no payload but firmware sends it separately)
            break
        # If firmware sends DATA + EOF in same REC GET when reaching end, we're done
        if len(buf) >= file_size:
            break
    print(f"\r  {file_size}/{file_size} bytes (100%)")
    return bytes(buf)


# ── Parse SD binary file ──────────────────────────────────────────────────────

CHANNEL_NAMES = ["ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
                 "qw", "qx", "qy", "qz", "dio"]

def parse_sd_file(data: bytes) -> tuple[dict, list[dict]]:
    if len(data) < SDLOG_HEADER_SIZE:
        raise ValueError("File too small to contain SD log header")

    (magic, version, record_size, sample_hz,
     channel_count, start_time_us) = struct.unpack_from(SDLOG_HEADER_FMT, data, 0)

    if magic != SD_LOG_MAGIC:
        raise ValueError(f"Bad SD log magic: {magic:#010x} (expected {SD_LOG_MAGIC:#010x})")

    header_info = {
        "version":       version,
        "record_size":   record_size,
        "sample_hz":     sample_hz,
        "channel_count": channel_count,
        "start_time_us": start_time_us,
    }

    rec_fmt = f"<IQ{channel_count}h"
    expected_rec_size = struct.calcsize(rec_fmt)
    if expected_rec_size != record_size:
        raise ValueError(
            f"Record size mismatch: header says {record_size}, "
            f"struct says {expected_rec_size} for {channel_count} channels"
        )

    names = CHANNEL_NAMES[:channel_count]
    if len(names) < channel_count:
        names += [f"ch{i}" for i in range(len(names), channel_count)]

    records = []
    offset = SDLOG_HEADER_SIZE
    while offset + record_size <= len(data):
        fields = struct.unpack_from(rec_fmt, data, offset)
        seq, time_us = fields[0], fields[1]
        channels = fields[2:]
        rec = {"seq": seq, "time_us": time_us}
        for name, val in zip(names, channels):
            rec[name] = val
        records.append(rec)
        offset += record_size

    return header_info, records


# ── Write CSV ─────────────────────────────────────────────────────────────────

def write_csv(path: str, header_info: dict, records: list[dict]) -> None:
    ch_count = header_info["channel_count"]
    col_names = ["seq", "time_us"] + CHANNEL_NAMES[:ch_count]
    if ch_count > len(CHANNEL_NAMES):
        col_names += [f"ch{i}" for i in range(len(CHANNEL_NAMES), ch_count)]

    with open(path, "w", newline="") as f:
        f.write(",".join(col_names) + "\n")
        for rec in records:
            row = [str(rec.get(c, 0)) for c in col_names]
            f.write(",".join(row) + "\n")
    print(f"  Wrote {len(records)} records → {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Download ESP32 SD recording via REC protocol")
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM3)")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--out",  default="recording.csv", help="Output CSV path")
    ap.add_argument("--no-verify", action="store_true", help="Skip final CRC32 check")
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        # 1. Get session metadata
        print("Querying session metadata...")
        meta = rec_session(ser)
        print(f"  session_id   : {meta['session_id']}")
        print(f"  file_size    : {meta['file_size']} bytes")
        print(f"  sample_count : {meta['sample_count']}")
        print(f"  file_checksum: {meta['file_checksum']:#010x}")

        if meta["file_size"] == 0:
            print("ERROR: file_size=0. Is a session finalized on the device?")
            sys.exit(1)

        # 2. Download
        print("Downloading...")
        raw = download_file(ser, meta["file_size"])

        # 3. Verify CRC
        if not args.no_verify:
            actual_crc = crc32(raw)
            if actual_crc != meta["file_checksum"]:
                print(f"WARNING: CRC32 mismatch — expected {meta['file_checksum']:#010x}, got {actual_crc:#010x}")
            else:
                print(f"  CRC32 OK ({actual_crc:#010x})")

        # 4. Acknowledge
        send(ser, "REC COMPLETE")

        # 5. Parse and write CSV
        print("Parsing SD binary file...")
        header_info, records = parse_sd_file(raw)
        print(f"  sample_hz    : {header_info['sample_hz']}")
        print(f"  channel_count: {header_info['channel_count']}")
        print(f"  records found: {len(records)}")
        write_csv(args.out, header_info, records)


if __name__ == "__main__":
    main()
