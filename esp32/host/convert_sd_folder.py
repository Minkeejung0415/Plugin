#!/usr/bin/env python3
"""Convert every STEP SD step_*.bin file in a folder/drive to CSV.

Examples:
  python esp32/host/convert_sd_folder.py D:/
  python esp32/host/convert_sd_folder.py D:/ --out C:/tmp/step_csv
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sd_bin_to_csv import convert_sd_bin_to_csv  # noqa: E402


def _dio_edge_times_from_csv(csv_path: Path) -> list[int]:
    edges: list[int] = []
    prev_level = None
    prev_edges = None
    with csv_path.open(newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                time_us = int(row["time_us"])
                dio = int(row["dio"])
            except (KeyError, TypeError, ValueError):
                continue
            level = dio & 1
            count = (dio >> 1) & 0x7FFF
            if prev_level is not None and (level != prev_level or count != prev_edges):
                edges.append(time_us)
            prev_level = level
            prev_edges = count
    return edges


def _sync_summary(converted: list[dict[str, Any]]) -> dict[str, Any]:
    edge_sets = []
    for item in converted:
        csv_path = Path(item["csv_path"])
        edge_sets.append({"path": str(csv_path), "edges_us": _dio_edge_times_from_csv(csv_path)})

    usable = [e for e in edge_sets if len(e["edges_us"]) >= 2]
    if len(usable) < 2:
        return {
            "status": "not_enough_edges",
            "message": "Need at least two files with two or more DIO edges to estimate offset/drift.",
            "files": edge_sets,
        }

    ref = usable[0]
    pairs = []
    for other in usable[1:]:
        n = min(len(ref["edges_us"]), len(other["edges_us"]))
        first_offset = other["edges_us"][0] - ref["edges_us"][0]
        last_offset = other["edges_us"][n - 1] - ref["edges_us"][n - 1]
        pairs.append({
            "reference": ref["path"],
            "other": other["path"],
            "matched_edges": n,
            "first_offset_us": first_offset,
            "last_offset_us": last_offset,
            "drift_us": last_offset - first_offset,
        })
    return {"status": "ok", "pairs": pairs, "files": edge_sets}


def convert_folder(root: Path, out_dir: Path | None = None) -> dict[str, Any]:
    if not root.exists():
        raise ValueError(f"Not found: {root}")
    if root.is_file():
        files = [root]
        base_dir = root.parent
    else:
        files = sorted(root.rglob("step_*.bin"))
        base_dir = root
    if not files:
        raise ValueError(f"No step_*.bin files found under {root}")

    target_dir = out_dir if out_dir is not None else base_dir
    target_dir.mkdir(parents=True, exist_ok=True)

    converted: list[dict[str, Any]] = []
    for bin_path in files:
        stem = bin_path.stem
        csv_path = target_dir / f"{stem}.csv"
        summary_path = target_dir / f"{stem}_summary.json"
        summary = convert_sd_bin_to_csv(bin_path, csv_path, summary_path)
        converted.append(summary)

    summary_csv = target_dir / "conversion_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "path", "csv_path", "record_count", "duration_s", "sample_hz",
            "first_seq", "last_seq", "lost_sequence_count", "duplicate_sequence_count",
            "quaternion_nonzero_percent", "dio_edge_count", "sync_usable",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for item in converted:
            writer.writerow(item)

    sync = _sync_summary(converted)
    folder_summary = {
        "root": str(root),
        "out_dir": str(target_dir),
        "file_count": len(converted),
        "summary_csv": str(summary_csv),
        "files": converted,
        "sync": sync,
    }
    (target_dir / "conversion_summary.json").write_text(json.dumps(folder_summary, indent=2), encoding="utf-8")
    return folder_summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="SD root, folder, or single step_*.bin file")
    parser.add_argument("--out", type=Path, default=None, help="Output folder for CSV and summaries")
    args = parser.parse_args()

    try:
        result = convert_folder(args.root, args.out)
    except ValueError as exc:
        print(exc)
        return 1

    print(f"Converted {result['file_count']} file(s)")
    print(f"Output: {result['out_dir']}")
    print(f"Summary: {result['summary_csv']}")
    print(f"Sync: {result['sync']['status']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
