#!/usr/bin/env python3
"""Automated serial sample-rate stress sweep for STEP ESP32 bench firmware."""
from __future__ import annotations

import argparse
import itertools
import json
import re
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import platform

try:
    import serial
    import serial.tools.list_ports
    from serial.serialutil import SerialException
except ModuleNotFoundError:
    serial = None  # type: ignore[assignment]
    SerialException = OSError  # type: ignore[assignment,misc]


ROOT = Path(__file__).resolve().parent
ANALYZER = ROOT / "analyze_sample_rate.py"
RESULTS_DIR = ROOT / "stress_results"

DEFAULT_HZ = [50, 100, 150, 200, 250, 300, 400, 500, 750, 1000, 1500, 2000]
ESP32_HINTS = (
    "ch340",
    "cp210",
    "cp2102",
    "usb serial",
    "silicon labs",
    "usb jtag",
    "esp32",
    "uart",
    "enhanced com",
)

HEADER_FMT = "<iiHiii"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
DEFAULT_PAYLOAD_SIZE = 11 * 2  # NUM_CHANNELS * int16
DEFAULT_FRAME_SIZE = HEADER_SIZE + DEFAULT_PAYLOAD_SIZE

CSV_LINE = re.compile(r"^\s*(\d+)\s*,")




def format_port_busy_help(port: str) -> str:
    lines = [
        f"Could not open {port!r}: Access is denied (port busy or blocked).",
        "",
        "Common causes:",
        "  - Arduino IDE Serial Monitor left open on this COM port",
        "  - host\\serial_tcp_bridge.py or run_usb_plugin_bridge.ps1 still running",
        "  - Another Python process (stress test, logger, REPL)",
        "  - Cursor / VS Code serial monitor or debug extension",
        "  - Firmware still rebooting after flash/upload (wait 3-5 s)",
        "",
        "On ESP32-S3 native USB (VID 0x303A): upload briefly locks the port.",
        "Close the IDE monitor before running host tools.",
        "",
    ]
    if platform.system() == "Windows":
        lines.append(_windows_port_holder_report())
    lines.extend(
        [
            "",
            "Quick checks:",
            f'  python -c "import serial; s=serial.Serial({port!r},115200,timeout=1); s.close(); print(\"ok\")"',
            "  powershell -File host\\diagnose_com_port.ps1",
            f"  python host/stress_test_serial.py --port {port} --wait-s 5",
        ]
    )
    return "\n".join(lines)


def _windows_port_holder_report() -> str:
    """Document-only: processes that often hold serial ports (no kill)."""
    names = (
        "python",
        "python3",
        "Arduino",
        "Code",
        "Cursor",
        "putty",
        "PuTTY",
        "serial",
        "mobaxterm",
    )
    ps = (
        "Get-Process -ErrorAction SilentlyContinue | "
        "Where-Object { $_.ProcessName -match 'python|Arduino|Code|Cursor|putty|PuTTY|serial' } | "
        "Select-Object Id,ProcessName | Format-Table -AutoSize | Out-String -Width 200"
    )
    try:
        proc = subprocess.run(
            ["powershell", "-NoProfile", "-Command", ps],
            capture_output=True,
            text=True,
            timeout=8,
            encoding="utf-8",
            errors="replace",
        )
        body = (proc.stdout or proc.stderr or "").strip()
    except Exception as exc:  # noqa: BLE001
        body = f"(could not enumerate processes: {exc})"
    if not body:
        body = "(no matching processes found by name; port may be held by another app)"
    return (
        "Windows: processes that often hold COM ports (informational only):\n"
        + body
    )


def open_serial_port(port: str, baud: int, wait_s: float) -> serial.Serial:
    if serial is None:
        raise RuntimeError("pyserial is required for serial stress tests: python -m pip install pyserial")

    attempts = 10 if wait_s > 0 else 1
    interval = 0.5
    last: BaseException | None = None
    for attempt in range(1, attempts + 1):
        try:
            return serial.Serial(port, baud, timeout=0.05)
        except SerialException as exc:
            last = exc
            msg = str(exc).lower()
            busy = "denied" in msg or "access" in msg or "permission" in msg
            if not busy or attempt >= attempts:
                break
            time.sleep(interval)
    print(format_port_busy_help(port), file=sys.stderr)
    assert last is not None
    raise last


@dataclass
class RateResult:
    hz: int
    filter_on: bool
    sd_on: bool
    mean_hz: float | None
    dup_seq: int
    gap_seq: int
    rows: int
    passed: bool
    result: str
    gap_time: int = 0
    sd_saved: int | None = None
    sd_errors: int | None = None
    max_sd_write_us: int | None = None
    max_loop_us: int | None = None
    loop_overruns: int | None = None
    note: str = ""


def list_ports_verbose() -> list[object]:
    if serial is None:
        return []
    return list(serial.tools.list_ports.comports())


def pick_port(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    ports = list_ports_verbose()
    if not ports:
        return None
    scored: list[tuple[int, str]] = []
    for p in ports:
        blob = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
        score = 0
        for hint in ESP32_HINTS:
            if hint in blob:
                score += 10
        if p.device.upper().startswith("COM"):
            score += 1
        scored.append((score, p.device))
    scored.sort(key=lambda x: (-x[0], x[1]))
    return scored[0][1]


def drain_serial(ser: serial.Serial, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            ser.read(n)
        else:
            time.sleep(0.02)


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line.rstrip("\n") + "\n").encode("ascii", errors="replace"))
    ser.flush()


def wait_for_text(ser: serial.Serial, timeout: float) -> str:
    buf = bytearray()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            text = buf.decode("utf-8", errors="replace")
            if (
                "ERROR" in text
                or "Sample rate set" in text
                or "OK FREQ" in text
                or "STATUS " in text
                or "SD_STATUS " in text
                or "OK FILTER" in text
                or "STOPPED" in text
            ):
                return text
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", errors="replace")


STATUS_RE = re.compile(r"(?:SD_STATUS|STATUS)\s+([^\r\n]*)")
SD_FINAL_RE = re.compile(r"SD_FINAL\s+([^\r\n]*)")


def parse_key_value_parts(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for part in text.split():
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        values[k.strip()] = v.strip()
    return values


def parse_key_values(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for m in STATUS_RE.finditer(text):
        values.update(parse_key_value_parts(m.group(1)))
    return values


def read_status(ser: serial.Serial, timeout: float = 1.0) -> dict[str, str]:
    send_line(ser, "STATUS")
    text = wait_for_text(ser, timeout)
    return parse_key_values(text)


def wait_for_sd_final(ser: serial.Serial, timeout: float) -> dict[str, str]:
    buf = bytearray()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            text = buf.decode("utf-8", errors="replace")
            m = SD_FINAL_RE.search(text)
            if m:
                return parse_key_value_parts(m.group(1))
        else:
            time.sleep(0.02)
    return {}


def maybe_int(values: dict[str, str], key: str) -> int | None:
    try:
        return int(values[key])
    except (KeyError, ValueError):
        return None


def first_int(values: dict[str, str], *keys: str) -> int | None:
    for key in keys:
        value = maybe_int(values, key)
        if value is not None:
            return value
    return None


def capture_text_csv(ser: serial.Serial, seconds: float, out_path: Path) -> int:
    end = time.monotonic() + seconds
    lines: list[str] = []
    buf = ""
    while time.monotonic() < end:
        raw = ser.read(ser.in_waiting or 1)
        if not raw:
            time.sleep(0.001)
            continue
        buf += raw.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if CSV_LINE.match(line):
                lines.append(line)
    out_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    return len(lines)


def capture_binary_to_csv(ser: serial.Serial, seconds: float, out_path: Path) -> tuple[int, float | None, int]:
    """Parse Open Ephys binary frames; write seq,time_s (hw_us when offset!=0 else host)."""
    end = time.monotonic() + seconds
    pending = bytearray()
    rows: list[str] = []
    host_t0: float | None = None
    last_host_s: float | None = None
    last_hw_us32: int | None = None
    hw_t0_us: int | None = None
    dts: list[float] = []
    prev_payload: bytes | None = None
    dup_payload: int = 0

    while time.monotonic() < end:
        chunk = ser.read(max(ser.in_waiting, 256))
        if chunk:
            pending.extend(chunk)
        else:
            time.sleep(0.001)

        while len(pending) >= HEADER_SIZE:
            frame_start = None
            header = None
            for i in range(0, len(pending) - HEADER_SIZE + 1):
                try:
                    candidate = struct.unpack_from(HEADER_FMT, pending, i)
                except struct.error:
                    break
                _offset, num_bytes, bit_depth, element_size, num_channels, samples_per_channel = candidate
                expected_bytes = num_channels * samples_per_channel * element_size
                if (
                    num_bytes == expected_bytes
                    and 2 <= num_bytes <= 128
                    and bit_depth in (3, 16)
                    and element_size == 2
                    and 1 <= num_channels <= 32
                    and samples_per_channel == 1
                ):
                    frame_start = i
                    header = candidate
                    break
            if frame_start is None or header is None:
                del pending[: max(0, len(pending) - HEADER_SIZE + 1)]
                break

            if frame_start > 0:
                del pending[:frame_start]

            offset, num_bytes, *_rest = header
            frame_size = HEADER_SIZE + num_bytes
            if len(pending) < frame_size:
                break
            payload = bytes(pending[HEADER_SIZE:frame_size])
            if prev_payload is not None and payload == prev_payload:
                dup_payload += 1
            prev_payload = payload
            del pending[:frame_size]
            seq = len(rows)
            if offset != 0:
                hw_us32 = offset & 0xFFFFFFFF
                if hw_t0_us is None:
                    hw_t0_us = hw_us32
                if last_hw_us32 is not None:
                    dts.append(((hw_us32 - last_hw_us32) & 0xFFFFFFFF) / 1e6)
                last_hw_us32 = hw_us32
                t_sec = (hw_us32 - hw_t0_us) / 1e6
            else:
                now = time.perf_counter()
                if host_t0 is None:
                    host_t0 = now
                t_sec = now - host_t0
                if last_host_s is not None:
                    dts.append(t_sec - last_host_s)
                last_host_s = t_sec
            rows.append(f"{seq},{t_sec:.6f}")

    out_path.write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")
    mean_hz = None
    if dts:
        mean_dt = sum(dts) / len(dts)
        if mean_dt > 0:
            mean_hz = 1.0 / mean_dt
    return len(rows), mean_hz, dup_payload


def run_analyzer(path: Path, time_col: int | None = None) -> tuple[int, int, float | None, int]:
    if not path.is_file() or path.stat().st_size == 0:
        return 0, 0, None, 0
    cmd = [sys.executable, str(ANALYZER), str(path)]
    if time_col is not None:
        cmd.extend(["--time-col", str(time_col)])
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    out = proc.stdout + proc.stderr
    dup = gap = gap_time = 0
    mean_hz = None
    m = re.search(r"Duplicate sequence values \(consecutive\): (\d+)", out)
    if m:
        dup = int(m.group(1))
    m = re.search(r"Lost samples \(seq jumps > 1\): (\d+)", out)
    if m:
        gap = int(m.group(1))
    m = re.search(r"Timestamp gaps \(dt > [^)]+\): (\d+)", out)
    if m:
        gap_time = int(m.group(1))
    m = re.search(r"Mean rate \(from timestamp\): ([0-9.]+) Hz", out)
    if m:
        mean_hz = float(m.group(1))
    return dup, gap, mean_hz, gap_time


def test_one_rate(
    ser: serial.Serial,
    hz: int,
    capture_s: float,
    binary_mode: bool | None,
    filter_on: bool,
    sd_on: bool,
) -> tuple[RateResult, bool | None]:
    send_line(ser, "FILTER ON" if filter_on else "FILTER OFF")
    time.sleep(0.15)
    send_line(ser, f"FREQ:{hz}")
    wait_for_text(ser, 2.0)
    drain_serial(ser, 0.2)

    if sd_on:
        send_line(ser, "RECORD ON")
        time.sleep(0.25)
        drain_serial(ser, 0.1)

    send_line(ser, "START")
    time.sleep(0.1)

    mode_tag = f"filter_{'on' if filter_on else 'off'}__sd_{'on' if sd_on else 'off'}"
    out_path = RESULTS_DIR / f"rate_{hz}__{mode_tag}.csv"
    dup_payload = 0
    if binary_mode is False:
        rows = capture_text_csv(ser, capture_s, out_path)
        dup, gap, mean_hz, gap_time = run_analyzer(out_path)
        mode = False
    else:
        rows, mean_hz_bin, dup_payload = capture_binary_to_csv(ser, capture_s, out_path)
        _, gap, mean_hz_a, gap_time = run_analyzer(out_path, time_col=1)
        mean_hz = mean_hz_bin or mean_hz_a
        dup = dup_payload
        mode = True
        expected = int(hz * capture_s * 0.85)
        if rows < expected:
            gap += max(0, int(hz * capture_s) - rows)

    status: dict[str, str] = {}
    sd_final_seen = False
    if sd_on:
        send_line(ser, "STOP")
        wait_for_text(ser, 2.0)
        drain_serial(ser, 0.25)
        send_line(ser, "RECORD OFF")
        final_status = wait_for_sd_final(ser, 20.0)
        sd_final_seen = bool(final_status)
        status.update(read_status(ser, 1.0))
        status.update(final_status)
    else:
        status.update(read_status(ser, 1.0))

    expected = int(hz * capture_s * 0.85)
    rate_ok = mean_hz is not None and (0.95 * hz <= mean_hz <= 1.15 * hz)
    sd_saved = first_int(status, "saved", "sd_saved")
    sd_errors = first_int(status, "errors", "sd_errors")
    max_sd_write_us = maybe_int(status, "max_sd_write_us")
    max_loop_us = maybe_int(status, "max_loop_us")
    loop_overruns = first_int(status, "overrun", "loop_overruns")
    sd_ok = True
    if sd_on:
        sd_ok = (
            sd_final_seen
            and
            sd_saved is not None
            and sd_saved >= expected
            and (sd_errors or 0) == 0
            and (loop_overruns or 0) == 0
        )
    passed = dup == 0 and gap == 0 and rows >= expected and rate_ok and sd_ok
    result = "PASS" if passed else "FAIL"
    note = ""
    if binary_mode is None:
        note = "auto"
    if sd_on and not sd_final_seen:
        note = (note + "; " if note else "") + "missing SD_FINAL"
        result = "UNKNOWN"
    if rows < 2:
        note = (note + "; " if note else "") + "insufficient rows"
    if sd_on and sd_saved is None:
        note = (note + "; " if note else "") + "no SD counters"
    if sd_on and sd_errors:
        note = (note + "; " if note else "") + f"sd_errors={sd_errors}"
    if sd_on and loop_overruns:
        note = (note + "; " if note else "") + f"loop_overruns={loop_overruns}"
    if binary_mode is not False and dup_payload:
        note = (note + "; " if note else "") + f"dup_payload={dup_payload}"
    return (
        RateResult(
            hz=hz,
            filter_on=filter_on,
            sd_on=sd_on,
            mean_hz=mean_hz,
            dup_seq=dup,
            gap_seq=gap,
            rows=rows,
            passed=passed,
            result=result,
            gap_time=gap_time,
            sd_saved=sd_saved,
            sd_errors=sd_errors,
            max_sd_write_us=max_sd_write_us,
            max_loop_us=max_loop_us,
            loop_overruns=loop_overruns,
            note=note,
        ),
        mode,
    )


def expand_mode(value: str) -> list[bool]:
    if value == "both":
        return [False, True]
    return [value == "on"]


def detect_mode(ser: serial.Serial) -> bool:
    """Return True if binary frames dominate in a short sniff."""
    send_line(ser, "FREQ:100")
    time.sleep(0.2)
    sample = ser.read(4096)
    if b"Format: CSV" in sample or CSV_LINE.search(sample.decode("utf-8", errors="replace")):
        return False
    if sample.count(bytes([0, 0, 0, 0])) > 5 and HEADER_SIZE in range(20, 24):
        return True
    text = sample.decode("utf-8", errors="replace")
    if "Open Ephys binary" in text or "BIN:" in text:
        return True
    return True  # default firmware USB_OPEN_EPHYS_MODE


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", help="Serial port (e.g. COM3)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--capture-s", type=float, default=8.0, help="Seconds per rate (max total bounded)")
    p.add_argument("--hz", type=int, nargs="*", default=DEFAULT_HZ)
    p.add_argument("--csv-mode", action="store_true", help="Force text CSV parsing")
    p.add_argument("--binary-mode", action="store_true", help="Force binary frame parsing")
    p.add_argument(
        "--filter",
        choices=("on", "off", "both"),
        default="both",
        help="Filter mode sweep. Use both to measure filter CPU cost.",
    )
    p.add_argument(
        "--sd",
        choices=("on", "off", "both"),
        default="both",
        help="SD recording mode sweep. Use both to isolate SD write cost.",
    )
    p.add_argument(
        "--sd-file",
        type=Path,
        help="Analyze a copied STEP SD binary file and exit.",
    )
    p.add_argument("--list-ports", action="store_true")
    p.add_argument(
        "--wait-s",
        type=float,
        default=0,
        help="Retry open up to 10 times (0.5 s apart) when port is busy; e.g. 5",
    )
    args = p.parse_args()

    if args.sd_file:
        cmd = [sys.executable, str(ANALYZER), str(args.sd_file), "--format", "sd-bin"]
        return subprocess.run(cmd).returncode

    if args.list_ports:
        ports = list_ports_verbose()
        if not ports:
            print("No serial ports found.")
            return 1
        espressif = [x for x in ports if x.vid == 0x303A]
        if espressif:
            print("# Espressif (VID 0x303A) - use CDC USB Serial for bench @115200:")
            for x in espressif:
                print(f"  {x.device}\t{x.description}\tpid=0x{(x.pid or 0):04X}\t{x.hwid}")
            print("# If multiple 303A ports: use the one that shows boot text in Serial Monitor.")
        for x in ports:
            tag = " [303A]" if x.vid == 0x303A else ""
            print(f"{x.device}\t{x.description}{tag}\t{x.hwid}")
        return 0

    port = pick_port(args.port)
    if not port:
        print("No serial ports detected. Connect ESP32 USB and re-run.")
        print("  python host/stress_test_serial.py --list-ports")
        return 2

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    per_test = min(args.capture_s, 120.0)
    if per_test * len(args.hz) > 600:
        print("Warning: many frequencies; keep capture-s modest.", file=sys.stderr)

    print(f"Using port {port} @ {args.baud}")
    results: list[RateResult] = []
    binary_mode: bool | None = None
    if args.csv_mode:
        binary_mode = False
    elif args.binary_mode:
        binary_mode = True

    try:
        ser = open_serial_port(port, args.baud, args.wait_s)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 3
    except SerialException:
        return 3
    try:
        time.sleep(2.0)
        drain_serial(ser, 1.0)
        if binary_mode is None:
            binary_mode = detect_mode(ser)
            print(f"Detected {'binary' if binary_mode else 'CSV'} serial stream")

        modes = list(itertools.product(expand_mode(args.filter), expand_mode(args.sd)))
        for filter_on, sd_on in modes:
            print(f"\n=== Mode: filter={'on' if filter_on else 'off'} sd={'on' if sd_on else 'off'} ===")
            for hz in args.hz:
                t_start = time.monotonic()
                res, binary_mode = test_one_rate(
                    ser, hz, per_test, binary_mode, filter_on=filter_on, sd_on=sd_on
                )
                results.append(res)
                elapsed = time.monotonic() - t_start
                if elapsed > per_test + 30:
                    print(f"Stopping early: {hz} Hz exceeded {per_test + 30:.0f} s")
                    break
                print(
                    f"Hz={hz}: rows={res.rows} mean_hz={res.mean_hz} dup={res.dup_seq} "
                    f"gap={res.gap_seq} sd_saved={res.sd_saved} sd_err={res.sd_errors} "
                    f"max_sd_us={res.max_sd_write_us} overrun={res.loop_overruns} pass={res.result}"
                )

        print("\n| Hz | filter | sd | mean_hz | rows | dup_seq | gap_seq | sd_saved | sd_err | max_sd_us | overrun | pass |")
        print("|----|--------|----|---------|------|---------|---------|----------|--------|-----------|---------|------|")
        last_good = 0
        for r in results:
            mh = f"{r.mean_hz:.1f}" if r.mean_hz is not None else "n/a"
            pf = r.result
            print(
                f"| {r.hz} | {'on' if r.filter_on else 'off'} | {'on' if r.sd_on else 'off'} | "
                f"{mh} | {r.rows} | {r.dup_seq} | {r.gap_seq} | "
                f"{r.sd_saved if r.sd_saved is not None else 'n/a'} | "
                f"{r.sd_errors if r.sd_errors is not None else 'n/a'} | "
                f"{r.max_sd_write_us if r.max_sd_write_us is not None else 'n/a'} | "
                f"{r.loop_overruns if r.loop_overruns is not None else 'n/a'} | {pf} |"
            )
            if r.passed:
                last_good = r.hz

        recommended = int(last_good * 0.8) if last_good else 0
        print(f"\nHighest passing Hz: {last_good}")
        print(f"Recommended cap (80%): {recommended} Hz")

        summary = RESULTS_DIR / "SUMMARY.md"
        lines = [
            "# Stress sweep summary",
            "",
            f"Port: `{port}` baud {args.baud}",
            "",
            "| Hz | filter | sd | mean_hz | rows | dup_seq | gap_seq | sd_saved | sd_err | max_sd_us | overrun | pass |",
            "|----|--------|----|---------|------|---------|---------|----------|--------|-----------|---------|------|",
        ]
        for r in results:
            mh = f"{r.mean_hz:.1f}" if r.mean_hz is not None else "n/a"
            lines.append(
                f"| {r.hz} | {'on' if r.filter_on else 'off'} | {'on' if r.sd_on else 'off'} | "
                f"{mh} | {r.rows} | {r.dup_seq} | {r.gap_seq} | "
                f"{r.sd_saved if r.sd_saved is not None else 'n/a'} | "
                f"{r.sd_errors if r.sd_errors is not None else 'n/a'} | "
                f"{r.max_sd_write_us if r.max_sd_write_us is not None else 'n/a'} | "
                f"{r.loop_overruns if r.loop_overruns is not None else 'n/a'} | "
                f"{r.result} |"
            )
        lines.extend(
            [
                "",
                "Pass rule: `mean_hz` must be within **95%–115%** of requested Hz "
                "(e.g. 1500 Hz @ 1320 Hz → **FAIL**; USB/loop ceiling ~1.3 kHz on this path).",
                "",
                f"Highest passing Hz: **{last_good}**",
                f"Recommended cap (80% of highest pass): **{recommended} Hz**",
            ]
        )
        summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
        json_path = RESULTS_DIR / "SUMMARY.json"
        json_path.write_text(
            json.dumps([r.__dict__ for r in results], indent=2) + "\n",
            encoding="utf-8",
        )
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
