#!/usr/bin/env python3
"""Convert a STEP ESP32 SD binary recording to CSV and summarize quality.

Examples:
  python esp32/host/sd_bin_to_csv.py D:/step_000174f4000020ea.bin
  python esp32/host/sd_bin_to_csv.py D:/step_000174f4000020ea.bin --summary-json
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
import struct
from pathlib import Path
from typing import Any


SD_LOG_MAGIC = 0x31505453  # "STP1" little-endian
SDLOG_HEADER_FMT = "<IHHHHq"
SDLOG_HEADER_SIZE = struct.calcsize(SDLOG_HEADER_FMT)
CHANNEL_NAMES = [
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
    "qw", "qx", "qy", "qz",
    "dio",
]


def _read_header(data: bytes, input_path: Path) -> tuple[int, int, int, int]:
    if len(data) < SDLOG_HEADER_SIZE:
        raise ValueError(f"{input_path}: too small for STEP SD header")

    magic, version, record_size, sample_hz, channel_count, start_time_us = struct.unpack_from(
        SDLOG_HEADER_FMT, data, 0
    )
    if magic != SD_LOG_MAGIC:
        raise ValueError(f"{input_path}: bad SD magic 0x{magic:08x}, expected 0x{SD_LOG_MAGIC:08x}")
    if version != 1:
        raise ValueError(f"{input_path}: unsupported SD log version {version}")
    if channel_count != 14:
        raise ValueError(f"{input_path}: expected 14 channels, got {channel_count}")

    expected_record_size = struct.calcsize(f"<Iq{channel_count}h")
    if record_size != expected_record_size:
        raise ValueError(
            f"{input_path}: record_size={record_size}, expected {expected_record_size} "
            f"for {channel_count} channels"
        )
    return record_size, sample_hz, channel_count, start_time_us


def _summarize_records(
    input_path: Path,
    record_size: int,
    sample_hz: int,
    channel_count: int,
    start_time_us: int,
    rows: list[tuple[int, int, tuple[int, ...]]],
    trailing_bytes: int,
) -> dict[str, Any]:
    seqs = [r[0] for r in rows]
    times = [r[1] for r in rows]
    dt_us = [b - a for a, b in zip(times, times[1:]) if b > a]
    seq_deltas = [b - a for a, b in zip(seqs, seqs[1:])]

    lost = sum(max(0, d - 1) for d in seq_deltas)
    dup = sum(1 for d in seq_deltas if d == 0)
    duration_s = ((times[-1] - times[0]) / 1_000_000.0) if len(times) >= 2 else 0.0
    expected = int(round(duration_s * sample_hz)) + 1 if duration_s > 0 else len(rows)

    quat_nonzero = 0
    dio_edges = 0
    first_edge_us = None
    last_edge_us = None
    prev_dio_level = None
    prev_dio_edges = None
    for seq, time_us, ch in rows:
        if any(ch[i] != 0 for i in range(9, 13)):
            quat_nonzero += 1
        dio = ch[13]
        dio_level = dio & 1
        dio_count = (dio >> 1) & 0x7FFF
        changed = (prev_dio_level is not None and dio_level != prev_dio_level)
        counted = (prev_dio_edges is not None and dio_count != prev_dio_edges)
        if changed or counted:
            dio_edges += max(1, dio_count - prev_dio_edges) if counted and dio_count >= prev_dio_edges else 1
            first_edge_us = time_us if first_edge_us is None else first_edge_us
            last_edge_us = time_us
        prev_dio_level = dio_level
        prev_dio_edges = dio_count

    return {
        "path": str(input_path),
        "record_count": len(rows),
        "sample_hz": sample_hz,
        "channel_count": channel_count,
        "record_size": record_size,
        "start_time_us": start_time_us,
        "duration_s": duration_s,
        "first_seq": seqs[0] if seqs else None,
        "last_seq": seqs[-1] if seqs else None,
        "expected_sample_count_from_time": expected,
        "lost_sequence_count": lost,
        "duplicate_sequence_count": dup,
        "mean_dt_us": (sum(dt_us) / len(dt_us)) if dt_us else None,
        "median_dt_us": statistics.median(dt_us) if dt_us else None,
        "trailing_bytes": trailing_bytes,
        "quaternion_nonzero_rows": quat_nonzero,
        "quaternion_nonzero_percent": (100.0 * quat_nonzero / len(rows)) if rows else 0.0,
        "dio_edge_count": dio_edges,
        "first_dio_edge_time_us": first_edge_us,
        "last_dio_edge_time_us": last_edge_us,
        "sync_usable": dio_edges >= 2,
    }


def convert_sd_bin_to_csv(input_path: Path, output_path: Path, summary_path: Path | None = None) -> dict[str, Any]:
    data = input_path.read_bytes()
    record_size, sample_hz, channel_count, start_time_us = _read_header(data, input_path)
    record_fmt = f"<Iq{channel_count}h"
    channel_names = CHANNEL_NAMES[:channel_count]

    payload = data[SDLOG_HEADER_SIZE:]
    record_count = len(payload) // record_size
    trailing_bytes = len(payload) % record_size
    rows: list[tuple[int, int, tuple[int, ...]]] = []

    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["seq", "time_us", *channel_names])
        for i in range(record_count):
            offset = SDLOG_HEADER_SIZE + i * record_size
            seq, time_us, *channels = struct.unpack_from(record_fmt, data, offset)
            ch_tuple = tuple(int(v) for v in channels)
            rows.append((int(seq), int(time_us), ch_tuple))
            writer.writerow([seq, time_us, *ch_tuple])

    summary = _summarize_records(
        input_path, record_size, sample_hz, channel_count, start_time_us, rows, trailing_bytes
    )
    summary["csv_path"] = str(output_path)
    if summary_path is not None:
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="STEP SD .bin file copied from the card")
    parser.add_argument("--out", type=Path, default=None, help="CSV output path")
    parser.add_argument("--summary-json", type=Path, nargs="?", const=True, default=None,
                        help="Write summary JSON. Optional path; default is *_summary.json")
    args = parser.parse_args()

    if not args.input.is_file():
        print(f"Not found: {args.input}")
        return 1

    output = args.out if args.out is not None else args.input.with_suffix(".csv")
    summary_path = None
    if args.summary_json is True:
        summary_path = args.input.with_name(args.input.stem + "_summary.json")
    elif isinstance(args.summary_json, Path):
        summary_path = args.summary_json

    try:
        summary = convert_sd_bin_to_csv(args.input, output, summary_path)
    except ValueError as exc:
        print(exc)
        return 1

    print(f"Input: {args.input}")
    print(f"Output: {output}")
    print(f"Records: {summary['record_count']}  duration={summary['duration_s']:.3f}s")
    print(f"Lost seq: {summary['lost_sequence_count']}  duplicates: {summary['duplicate_sequence_count']}")
    print(f"Quaternion nonzero: {summary['quaternion_nonzero_percent']:.1f}%")
    print(f"DIO edges: {summary['dio_edge_count']}")
    if summary_path is not None:
        print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
