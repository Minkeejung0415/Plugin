#!/usr/bin/env python3
"""Analyze STEP CSV, serial bench logs, or SD binary logs for sample-rate tests."""
from __future__ import annotations

import argparse
import csv
import json
import struct
from dataclasses import asdict, dataclass
from pathlib import Path


SD_MAGIC_STP1 = 0x31505453
SD_HEADER_FMT = "<IHHHHq"
SD_HEADER_SIZE = struct.calcsize(SD_HEADER_FMT)


@dataclass
class AnalysisResult:
    path: str
    rows: int
    mean_hz: float | None
    median_dt_us: float | None
    dup_seq: int
    dup_row: int
    gap_seq: int
    gap_time: int
    first_seq: int | None
    last_seq: int | None
    sample_rate_hz: int | None = None
    channel_count: int | None = None
    record_size: int | None = None


def _parse_row(row: list[str], seq_col: int, t_col: int | None) -> tuple[int | None, float | None]:
    seq = None
    t = None
    try:
        if 0 <= seq_col < len(row):
            seq = int(float(row[seq_col].strip()))
    except ValueError:
        pass
    if t_col is not None and 0 <= t_col < len(row):
        try:
            t = float(row[t_col].strip())
        except ValueError:
            pass
    return seq, t


def _summarize(
    path: Path,
    rows: list[tuple[int | None, float | None, object]],
    gap_factor: float,
    sample_rate_hz: int | None = None,
    channel_count: int | None = None,
    record_size: int | None = None,
) -> AnalysisResult:
    dup_seq = 0
    dup_row = 0
    gap_seq = 0
    gap_time = 0
    dt_list: list[float] = []

    for i in range(1, len(rows)):
        prev_seq, prev_t, prev_row = rows[i - 1]
        seq, t, row = rows[i]

        if prev_row == row:
            dup_row += 1

        if prev_seq is not None and seq is not None:
            if seq == prev_seq:
                dup_seq += 1
            elif seq > prev_seq + 1:
                gap_seq += seq - prev_seq - 1

        if prev_t is not None and t is not None:
            dt = t - prev_t
            if dt > 0:
                dt_list.append(dt)

    mean_hz = None
    median_dt_us = None
    if dt_list:
        median_dt = sorted(dt_list)[len(dt_list) // 2]
        median_dt_us = median_dt * 1e6
        mean_dt = sum(dt_list) / len(dt_list)
        if mean_dt > 0:
            mean_hz = 1.0 / mean_dt
        if median_dt > 0 and gap_factor > 0:
            threshold = gap_factor * median_dt
            gap_time = sum(1 for dt in dt_list if dt > threshold)

    seqs = [seq for seq, _t, _row in rows if seq is not None]
    return AnalysisResult(
        path=str(path),
        rows=len(rows),
        mean_hz=mean_hz,
        median_dt_us=median_dt_us,
        dup_seq=dup_seq,
        dup_row=dup_row,
        gap_seq=gap_seq,
        gap_time=gap_time,
        first_seq=seqs[0] if seqs else None,
        last_seq=seqs[-1] if seqs else None,
        sample_rate_hz=sample_rate_hz,
        channel_count=channel_count,
        record_size=record_size,
    )


def analyze_csv(path: Path, seq_col: int, t_col: int | None, gap_factor: float) -> AnalysisResult:
    rows: list[tuple[int | None, float | None, object]] = []
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            if row[0].lower() in ("seq", "sample", "index"):
                continue
            seq, t = _parse_row(row, seq_col, t_col)
            rows.append((seq, t, tuple(row)))

    if len(rows) < 2:
        raise ValueError(f"{path}: need at least 2 data rows (got {len(rows)})")
    return _summarize(path, rows, gap_factor)


def analyze_sd_bin(path: Path, gap_factor: float) -> AnalysisResult:
    data = path.read_bytes()
    if len(data) < SD_HEADER_SIZE:
        raise ValueError(f"{path}: too small for STEP SD header")

    magic, version, record_size, sample_hz, channel_count, _start_us = struct.unpack_from(
        SD_HEADER_FMT, data, 0
    )
    if magic != SD_MAGIC_STP1:
        raise ValueError(f"{path}: unsupported SD magic 0x{magic:08x}")
    if version != 1:
        raise ValueError(f"{path}: unsupported SD version {version}")
    if record_size < 12:
        raise ValueError(f"{path}: invalid SD record size {record_size}")

    payload = data[SD_HEADER_SIZE:]
    full_records = len(payload) // record_size
    trailing = len(payload) % record_size
    if trailing:
        print(f"Warning: truncated tail bytes: {trailing}")

    rows: list[tuple[int | None, float | None, object]] = []
    for i in range(full_records):
        offset = SD_HEADER_SIZE + i * record_size
        seq, time_us = struct.unpack_from("<Iq", data, offset)
        rows.append((seq, time_us / 1e6, (seq, time_us)))

    if len(rows) < 2:
        raise ValueError(f"{path}: need at least 2 SD records (got {len(rows)})")
    return _summarize(
        path,
        rows,
        gap_factor,
        sample_rate_hz=sample_hz,
        channel_count=channel_count,
        record_size=record_size,
    )


def print_result(result: AnalysisResult, has_time: bool, gap_factor: float) -> None:
    print(f"File: {result.path}")
    print(f"  Rows: {result.rows}")
    if result.sample_rate_hz is not None:
        print(f"  Header sample rate: {result.sample_rate_hz} Hz")
    if result.channel_count is not None:
        print(f"  Header channels: {result.channel_count}")
    if result.record_size is not None:
        print(f"  Header record size: {result.record_size} bytes")
    if result.mean_hz is not None:
        print(f"  Mean rate (from timestamp): {result.mean_hz:.2f} Hz")
    if result.median_dt_us is not None:
        print(f"  Median dt: {result.median_dt_us:.1f} us")
    print(f"  Sequence range: {result.first_seq}..{result.last_seq}")
    print(f"  Duplicate sequence values (consecutive): {result.dup_seq}")
    print(f"  Identical consecutive rows: {result.dup_row}")
    print(f"  Lost samples (seq jumps > 1): {result.gap_seq}")
    if has_time:
        print(f"  Timestamp gaps (dt > {gap_factor}x median): {result.gap_time}")
    else:
        print("  Timestamp gaps: (no time column - use --time-col)")

    if result.dup_seq or result.dup_row:
        print("  => Likely rate too high or host cannot keep up (duplicates).")
    if result.gap_seq or result.gap_time:
        print("  => Likely drops / scheduling gaps (lost samples).")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path, help="CSV or STEP SD binary log path")
    p.add_argument(
        "--format",
        choices=("auto", "csv", "sd-bin"),
        default="auto",
        help="Input format. auto detects STEP SD binary headers.",
    )
    p.add_argument("--seq-col", type=int, default=0, help="CSV sequence column index")
    p.add_argument("--time-col", type=int, default=None, help="CSV timestamp column index")
    p.add_argument("--gap-factor", type=float, default=2.0)
    p.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    args = p.parse_args()

    if not args.input.is_file():
        print(f"Not found: {args.input}")
        return 1

    try:
        fmt = args.format
        if fmt == "auto":
            head = args.input.read_bytes()[:4]
            fmt = (
                "sd-bin"
                if len(head) == 4 and struct.unpack("<I", head)[0] == SD_MAGIC_STP1
                else "csv"
            )

        if fmt == "sd-bin":
            result = analyze_sd_bin(args.input, args.gap_factor)
            has_time = True
        else:
            result = analyze_csv(args.input, args.seq_col, args.time_col, args.gap_factor)
            has_time = args.time_col is not None
    except ValueError as exc:
        print(str(exc))
        return 1

    if args.json:
        print(json.dumps(asdict(result), indent=2))
    else:
        print_result(result, has_time, args.gap_factor)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
