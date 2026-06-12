#!/usr/bin/env python3
"""Analyze STEP CSV or seq,t,... bench logs for sample-rate stress tests."""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


def _parse_row(row: list[str], seq_col: int, t_col: int | None) -> tuple[int | None, float | None]:
    seq = None
    t = None
    try:
        if seq_col >= 0 and seq_col < len(row):
            seq = int(float(row[seq_col].strip()))
    except ValueError:
        pass
    if t_col is not None and t_col >= 0 and t_col < len(row):
        try:
            t = float(row[t_col].strip())
        except ValueError:
            pass
    return seq, t


def analyze(path: Path, seq_col: int, t_col: int | None, gap_factor: float) -> int:
    rows: list[tuple[int | None, float | None, list[str]]] = []
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or (row[0].startswith("#")):
                continue
            if row[0].lower() in ("seq", "sample", "index"):
                continue
            seq, t = _parse_row(row, seq_col, t_col)
            rows.append((seq, t, row))

    if len(rows) < 2:
        print(f"{path}: need at least 2 data rows (got {len(rows)})")
        return 1

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
    median_dt = None
    if dt_list:
        median_dt = sorted(dt_list)[len(dt_list) // 2]
        mean_dt = sum(dt_list) / len(dt_list)
        if mean_dt > 0:
            mean_hz = 1.0 / mean_dt
        if median_dt > 0 and gap_factor > 0:
            thresh = gap_factor * median_dt
            gap_time = sum(1 for d in dt_list if d > thresh)

    print(f"File: {path}")
    print(f"  Rows: {len(rows)}")
    if mean_hz is not None:
        print(f"  Mean rate (from timestamp): {mean_hz:.2f} Hz")
    if median_dt is not None:
        print(f"  Median dt: {median_dt * 1e6:.1f} us")
    print(f"  Duplicate sequence values (consecutive): {dup_seq}")
    print(f"  Identical consecutive rows: {dup_row}")
    print(f"  Lost samples (seq jumps > 1): {gap_seq}")
    if t_col is not None:
        print(
            f"  Timestamp gaps (dt > {gap_factor}x median): {gap_time}"
        )
    else:
        print("  Timestamp gaps: (no time column — use --time-col)")

    if dup_seq or dup_row:
        print("  => Likely rate too high or host cannot keep up (duplicates).")
    if gap_seq or gap_time:
        print("  => Likely drops / scheduling gaps (lost samples).")

    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv", type=Path, help="CSV path (seq first column by default)")
    p.add_argument("--seq-col", type=int, default=0, help="Sequence column index")
    p.add_argument(
        "--time-col",
        type=int,
        default=None,
        help="Timestamp column (seconds or us); omit if only seq present",
    )
    p.add_argument(
        "--gap-factor",
        type=float,
        default=2.0,
        help="Flag dt gaps larger than this multiple of median dt",
    )
    args = p.parse_args()
    if not args.csv.is_file():
        print(f"Not found: {args.csv}", file=sys.stderr)
        return 1
    return analyze(args.csv, args.seq_col, args.time_col, args.gap_factor)


if __name__ == "__main__":
    raise SystemExit(main())
