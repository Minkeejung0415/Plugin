#!/usr/bin/env python3
"""
USB Serial → TCP bridge for Open Ephys.

Use when the PC cannot join the ESP32 Wi-Fi (STEP_ESP32 Soft AP or hotspot).
Board streams Open Ephys binary on USB; this script listens on localhost:5000.

Sketch preset — set `USB_OPEN_EPHYS_MODE true` in step_node.ino (or manually):
  ENABLE_TCP false
  ENABLE_SERIAL_BENCH true
  SERIAL_OUTPUT_BINARY true

Windows examples:
  pip install pyserial
  # Built-in Ephys Socket (no REDPITAYA/START):
  python host/serial_tcp_bridge.py COM5
  # Minkeejung0415/Plugin AcqBoard (REDPITAYA/START handshake):
  python host/serial_tcp_bridge.py COM5 --plugin
  # Or: host/run_usb_plugin_bridge.ps1 COM5

Open Ephys:
  Ephys Socket → 127.0.0.1:5000
  Plugin AcqBoard → Node IP 127.0.0.1:5000 (with --plugin)
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import os
import struct
import sys
import threading
import time
from collections import deque

logger = logging.getLogger(__name__)

HEADER = struct.Struct("<iiHiii")
HEADER_SIZE = HEADER.size
SDRF_HEADER_SIZE = 64
SDRF_MAGIC = b"SDRF"
SDRF_TYPE_EOF = 0x02
SDRF_TYPE_ABORT = 0x03
SDRF_TYPE_ERROR = 0x05
STARTED_REPLY = b"STARTED BIN:step_usb_bridge\n"
SENSORS_REPLY = b"SENSORS:0,ICM20948\n"
# Open Ephys Ephys Socket: OpenCV Mat depth enum (S16), not literal 16 bits.
OE_BIT_DEPTH_S16 = 3
FIRMWARE_BIT_DEPTH = 16  # step_node fillOeHeader() sends literal 16
DEFAULT_NUM_CHANNELS = int(os.environ.get("ESP32_NUM_CHANNELS", "14"))
FRAME_PAYLOAD = DEFAULT_NUM_CHANNELS * 2
FRAME_SIZE = HEADER_SIZE + FRAME_PAYLOAD
DEFAULT_FIRST_FRAME_TIMEOUT = 15.0
HANDSHAKE_PEEK_TIMEOUT = 0.3
PLUGIN_CMD_TIMEOUT = 120.0

# Last FREQ forwarded to firmware (USB path has no REDPITAYA text reply on serial).
_plugin_sample_rate_hz = 100

_RELAY_PREFIXES = (
    "FREQ:",
    "FREQ ",
    "CFG ",
    "FILTER",
    "STOP",
    "RECORD ",
    "STATUS",
    "REC ",
)


def _should_relay_plugin_command(line: str) -> bool:
    upper = line.upper()
    return any(upper.startswith(p) for p in _RELAY_PREFIXES)


def _parse_freq_hz(line: str) -> int | None:
    upper = line.upper()
    if upper.startswith("FREQ:"):
        try:
            return int(line.split(":", 1)[1].strip())
        except ValueError:
            return None
    if upper.startswith("FREQ "):
        try:
            return int(line.split(None, 1)[1].strip())
        except (ValueError, IndexError):
            return None
    return None


def _note_forwarded_plugin_command(line: str) -> None:
    global _plugin_sample_rate_hz
    hz = _parse_freq_hz(line)
    if hz is not None and hz >= 1:
        _plugin_sample_rate_hz = hz


def open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    ser = serial.Serial(port, baud, timeout=0.1)
    ser.reset_input_buffer()
    return ser


def pack_csv_row(fields: list[str], num_channels: int = DEFAULT_NUM_CHANNELS) -> bytes | None:
    """Convert CSV seq plus channel values to one Open Ephys int16 frame."""
    if num_channels <= 0 or len(fields) < 2:
        return None
    try:
        _seq = int(fields[0])
        ch = [int(v) for v in fields[1 : 1 + num_channels]]
    except ValueError:
        return None

    if len(ch) < num_channels:
        ch.extend([0] * (num_channels - len(ch)))

    payload_bytes = num_channels * 2
    hdr = HEADER.pack(0, payload_bytes, OE_BIT_DEPTH_S16, 2, num_channels, 1)
    return hdr + struct.pack(f"<{num_channels}h", *ch)


def payload_bytes_from_header(hdr: tuple) -> int | None:
    """Bytes in payload; must match Open Ephys num_bytes and fixed read size."""
    _off, num_bytes, _bit_depth, elem, n_ch, n_per = hdr
    if elem <= 0 or n_ch <= 0 or n_per <= 0:
        return None
    expected = n_ch * n_per * elem
    return expected if num_bytes == expected else None


def is_valid_header(hdr: tuple) -> bool:
    """Reject mis-synced headers before TCP (OE compareHeaders is strict)."""
    _off, _num_bytes, bit_depth, elem, n_ch, n_per = hdr
    if elem != 2 or n_ch > 256 or n_per > 65536:
        return False
    if bit_depth not in (OE_BIT_DEPTH_S16, FIRMWARE_BIT_DEPTH) and bit_depth > 6:
        return False
    expected = payload_bytes_from_header(hdr)
    return expected is not None and expected <= 4096


def header_fields(frame: bytes) -> str:
    if len(frame) < HEADER_SIZE:
        return f"<{len(frame)} bytes>"
    hdr = HEADER.unpack_from(frame, 0)
    return (
        f"offset={hdr[0]} num_bytes={hdr[1]} bit_depth={hdr[2]} "
        f"element_size={hdr[3]} n_channels={hdr[4]} n_samples_per_channel={hdr[5]}"
    )


def normalize_frame_for_oe(frame: bytes) -> bytes:
    """Ephys Socket expects bit_depth=3 (S16 enum), not legacy 16."""
    if len(frame) < HEADER_SIZE:
        return frame
    hdr = HEADER.unpack_from(frame, 0)
    if hdr[2] == OE_BIT_DEPTH_S16:
        return frame
    if hdr[2] == FIRMWARE_BIT_DEPTH:
        fixed = (hdr[0], hdr[1], OE_BIT_DEPTH_S16, hdr[3], hdr[4], hdr[5])
        return HEADER.pack(*fixed) + frame[HEADER_SIZE:]
    return frame

def _ascii_preview(data: bytes, limit: int = 80) -> str:
    text = data[:limit].decode("utf-8", errors="replace")
    return text.replace("\r", "\\r").replace("\n", "\\n")


def diagnose_serial_buffer(buf: bytearray, csv_mode: bool) -> str | None:
    """Return a hint when no Open Ephys frames are parsed yet."""
    if not buf:
        return (
            "no bytes from serial — wrong COM port, cable, or another app has the port open"
        )
    sample = bytes(buf[:512])
    text = sample.decode("utf-8", errors="replace")
    if "TCP listen" in text or "Client connected" in text:
        return (
            "firmware looks like Wi-Fi TCP mode (ENABLE_TCP true, ENABLE_SERIAL_BENCH false) — "
            "set USB_OPEN_EPHYS_MODE true in step_node.ino and re-flash"
        )
    if "Wi-Fi skipped" in text and "Serial bench active" not in text:
        return "serial bench disabled — set ENABLE_SERIAL_BENCH true and re-flash"
    if "Format: CSV" in text or (not csv_mode and "seq," in text):
        return (
            "CSV serial detected — flash with SERIAL_OUTPUT_BINARY true "
            "or run bridge with --csv"
        )
    if csv_mode and b"\x00" in sample[:64]:
        return (
            "binary-like data while --csv set — use default binary mode "
            "(SERIAL_OUTPUT_BINARY true on board)"
        )
    if "CSV/stream paused" in text or "BOOT DIAGNOSTICS" in text:
        return (
            "boot diagnostics only — wait for BOOT_CSV_DELAY_MS (5 s) after boot, "
            "then confirm sample frames with serial_bench_reader.py --binary"
        )
    return (
        f"received {len(buf)} bytes but no valid Open Ephys header — "
        f"first bytes: {_ascii_preview(sample)}"
    )



class SerialFrameSource:
    """Background thread: parse binary (or CSV) frames from USB serial."""

    def __init__(self, port: str, baud: int, csv_mode: bool) -> None:
        self._ser = open_serial(port, baud)
        self._csv_mode = csv_mode
        self._buf = bytearray()
        self._queue: deque[bytes] = deque(maxlen=256)
        self._response_queue: deque[bytes] = deque(maxlen=64)
        self._transfer_queue: deque[bytes] = deque(maxlen=64)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bytes_received = 0
        self._logged_first_chunk = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self._ser.close()

    def get_frame(self, timeout: float = 0.5) -> bytes | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._queue:
                    return self._queue.popleft()
            time.sleep(0.01)
        return None

    def get_response(self, timeout: float = 0.5) -> bytes | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._response_queue:
                    return self._response_queue.popleft()
            time.sleep(0.01)
        return None

    def get_transfer_frame(self, timeout: float = 0.5) -> bytes | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._transfer_queue:
                    return self._transfer_queue.popleft()
            time.sleep(0.01)
        return None

    def wait_for_frames(self, timeout: float) -> bool:
        """Block until at least one parsed frame is queued (does not dequeue)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._queue:
                    return True
            time.sleep(0.05)
        return False

    def drain(self) -> None:
        with self._lock:
            self._queue.clear()

    def drain_control(self) -> None:
        with self._lock:
            self._response_queue.clear()
            self._transfer_queue.clear()

    def diagnostic_hint(self) -> str | None:
        with self._lock:
            return diagnose_serial_buffer(self._buf, self._csv_mode)

    def write_line(self, line: str) -> None:
        self._ser.write(line.encode("ascii"))

    def _push_frame(self, frame: bytes) -> None:
        with self._lock:
            self._queue.append(frame)

    def _push_response(self, line: bytes) -> None:
        with self._lock:
            self._response_queue.append(line)

    def _push_transfer_frame(self, frame: bytes) -> None:
        with self._lock:
            self._transfer_queue.append(frame)

    def _run(self) -> None:
        while not self._stop.is_set():
            chunk = self._ser.read(4096)
            if chunk:
                self._bytes_received += len(chunk)
                if not self._logged_first_chunk:
                    self._logged_first_chunk = True
                    preview = chunk[:64]
                    logger.info(
                        "serial first %d bytes (hex): %s",
                        len(preview),
                        preview.hex(" "),
                    )
                    logger.info("serial first bytes (text): %s", _ascii_preview(chunk, 120))
                self._buf.extend(chunk)
            if self._csv_mode:
                self._parse_csv_lines()
            else:
                self._parse_binary()

    def _parse_csv_lines(self) -> None:
        while b"\n" in self._buf:
            line, _, rest = self._buf.partition(b"\n")
            self._buf = bytearray(rest)
            text = line.decode("utf-8", errors="replace").strip()
            if not text or text.startswith("#") or text.startswith("STEP"):
                continue
            if text.startswith("Format:") or "Wi-Fi" in text or text.startswith("ICM"):
                continue
            if text.startswith("seq,"):
                continue
            parts = text.split(",")
            frame = pack_csv_row(parts)
            if frame:
                self._push_frame(frame)

    def _parse_binary(self) -> None:
        while len(self._buf) >= HEADER_SIZE:
            if self._buf.startswith(b"REC ") or self._buf.startswith(b"SD_STATUS") or self._buf.startswith(b"STATUS"):
                if b"\n" not in self._buf:
                    break
                line, _, rest = self._buf.partition(b"\n")
                self._buf = bytearray(rest)
                self._push_response(bytes(line + b"\n"))
                continue
            if self._buf.startswith(SDRF_MAGIC):
                if len(self._buf) < SDRF_HEADER_SIZE:
                    break
                payload_len = struct.unpack_from("<I", self._buf, 36)[0]
                total = SDRF_HEADER_SIZE + payload_len
                if len(self._buf) < total:
                    break
                frame = bytes(self._buf[:total])
                del self._buf[:total]
                self._push_transfer_frame(frame)
                continue
            hdr = HEADER.unpack_from(self._buf, 0)
            _off, num_bytes, _bd, elem, _n_ch, _n_per = hdr
            if not is_valid_header(hdr):
                del self._buf[0]
                continue
            expected = payload_bytes_from_header(hdr)
            if expected is None:
                del self._buf[0]
                continue
            total = HEADER_SIZE + expected
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            del self._buf[:total]
            self._push_frame(frame)


async def read_line(reader: asyncio.StreamReader) -> str:
    line = await reader.readline()
    return line.decode("utf-8", errors="replace").strip()


def _first_command_line(peek: bytes) -> str:
    return peek.split(b"\n", 1)[0].decode("utf-8", errors="replace").strip()


async def send_plugin_handshake_replies(
    writer: asyncio.StreamWriter, num_ch: int, sample_rate_hz: int | None = None
) -> None:
    """Match esp32_tcp_client / step_node.ino plus Red Pitaya lines for older Plugin builds."""
    sr = sample_rate_hz if sample_rate_hz is not None else _plugin_sample_rate_hz
    writer.write(
        f"{num_ch} channels; sample_rate={sr}; node=esp32s3_arduino\n".encode()
    )
    writer.write(f"OK CHANNELS:{num_ch}\n".encode())
    await writer.drain()


async def send_plugin_start_replies(writer: asyncio.StreamWriter) -> None:
    writer.write(STARTED_REPLY)
    writer.write(SENSORS_REPLY)
    await writer.drain()


def forward_serial_command(source: SerialFrameSource, command: str) -> None:
    """Optional USB logging: firmware pollSerialCommands() accepts REDPITAYA/START."""
    line = command.rstrip("\r\n")
    source.write_line(line + "\n")


async def relay_rec_command(
    writer: asyncio.StreamWriter,
    source: SerialFrameSource,
    command: str,
    timeout: float = 3.0,
) -> None:
    """Relay one rec-v1 command and return text or SDRF frames to the TCP client."""
    loop = asyncio.get_event_loop()
    source.drain_control()
    forward_serial_command(source, command)
    upper = command.upper()

    if upper.startswith("REC GET"):
        first = True
        while True:
            response = await loop.run_in_executor(None, source.get_response, 0.05)
            if response is not None:
                writer.write(response)
                await writer.drain()
                return
            frame = await loop.run_in_executor(
                None, source.get_transfer_frame, timeout if first else 1.0
            )
            first = False
            if frame is None:
                writer.write(b"REC ERR code=sd_error retryable=true detail=bridge_timeout\n")
                await writer.drain()
                return
            writer.write(frame)
            await writer.drain()
            frame_type = frame[5] if len(frame) > 5 else 0
            if frame_type in (SDRF_TYPE_EOF, SDRF_TYPE_ABORT, SDRF_TYPE_ERROR):
                return

    def is_terminal_rec_response(response: bytes) -> bool:
        text = response.decode("utf-8", errors="replace").strip().upper()
        if upper.startswith("REC START"):
            return text.startswith("REC STARTED") or text.startswith("REC ERR")
        if upper.startswith("REC STOP"):
            return text.startswith("REC FINALIZED") or text.startswith("REC ERR")
        if upper.startswith("REC STATUS"):
            return text.startswith("REC STATUS_OK") or text.startswith("REC ERR")
        if upper.startswith("REC SESSION"):
            return text.startswith("REC SESSION_OK") or text.startswith("REC ERR")
        if upper.startswith("REC COMPLETE"):
            return text.startswith("REC COMPLETE_OK") or text.startswith("REC ERR")
        if upper.startswith("REC ABORT"):
            return text.startswith("REC ABORTED") or text.startswith("REC ERR")
        if upper.startswith("REC CLEAR"):
            return text.startswith("REC CLEAR_OK") or text.startswith("REC ERR")
        if upper.startswith("REC HELLO"):
            return text.startswith("REC HELLO_OK") or text.startswith("REC ERR")
        return text.startswith("REC ") or text.startswith("SD_STATUS")

    deadline = time.monotonic() + timeout
    saw_response = False
    while True:
        remaining = max(0.05, deadline - time.monotonic())
        response = await loop.run_in_executor(None, source.get_response, remaining)
        if response is None:
            if not saw_response:
                writer.write(b"REC ERR code=internal_error retryable=true detail=bridge_timeout\n")
                await writer.drain()
            return

        saw_response = True
        writer.write(response)
        await writer.drain()

        if is_terminal_rec_response(response):
            return

        if time.monotonic() >= deadline:
            return


async def handle_plugin_handshake(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    source: SerialFrameSource,
    first_frame_timeout: float,
    verbose: bool,
    verbose_frames: int,
    num_ch: int,
    initial_peek: bytes = b"",
) -> None:
    """REDPITAYA → channel replies; START → STARTED/SENSORS; then binary from serial."""
    first_line = _first_command_line(initial_peek) if initial_peek else ""
    if not first_line.upper().startswith("REDPITAYA"):
        line = await asyncio.wait_for(read_line(reader), timeout=PLUGIN_CMD_TIMEOUT)
        if not line.upper().startswith("REDPITAYA"):
            logger.warning("Plugin mode expected REDPITAYA, got: %r", line)
            return
        first_line = line

    logger.info("Plugin handshake REDPITAYA")
    forward_serial_command(source, "REDPITAYA")
    await send_plugin_handshake_replies(writer, num_ch)

    while True:
        line = await asyncio.wait_for(read_line(reader), timeout=PLUGIN_CMD_TIMEOUT)
        if not line:
            logger.warning("Plugin client closed before START")
            return
        if line.upper().startswith("START"):
            logger.info("Plugin START — streaming serial binary on TCP")
            forward_serial_command(source, "START")
            source.drain()
            await send_plugin_start_replies(writer)
            await stream_frames(
                writer,
                source,
                first_frame_timeout,
                verbose,
                verbose_frames,
                command_reader=reader,
            )
            return
        if _should_relay_plugin_command(line):
            logger.info("Plugin → serial (pre-START): %s", line)
            if line.upper().startswith("REC "):
                await relay_rec_command(writer, source, line)
            else:
                forward_serial_command(source, line)
                _note_forwarded_plugin_command(line)
            continue
        logger.debug("Plugin ignored command while waiting for START: %r", line)


async def handle_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    source: SerialFrameSource,
    first_frame_timeout: float,
    num_ch: int,
    plugin_mode: bool = False,
    verbose: bool = False,
    verbose_frames: int = 5,
) -> None:
    peer = writer.get_extra_info("peername")
    logger.info("TCP client connected: %s", peer)
    try:
        # Ephys Socket reads binary immediately; Plugin / esp32_tcp_client send REDPITAYA first.
        try:
            peek = await asyncio.wait_for(reader.read(256), timeout=HANDSHAKE_PEEK_TIMEOUT)
        except asyncio.TimeoutError:
            peek = b""

        if plugin_mode:
            logger.info("Plugin AcqBoard mode (--plugin)")
            await handle_plugin_handshake(
                reader,
                writer,
                source,
                first_frame_timeout,
                verbose,
                verbose_frames,
                num_ch,
                initial_peek=peek,
            )
            return

        if not peek:
            logger.info("Ephys Socket mode — streaming binary (no handshake)")
            await stream_frames(
                writer, source, first_frame_timeout, verbose, verbose_frames
            )
            return

        first_line = _first_command_line(peek)
        if first_line.upper().startswith("REC "):
            logger.info("rec-v1 command mode")
            await relay_rec_command(writer, source, first_line)
            while True:
                line = await read_line(reader)
                if not line:
                    break
                if line.upper().startswith("REC "):
                    await relay_rec_command(writer, source, line)
                elif _should_relay_plugin_command(line):
                    forward_serial_command(source, line)
                    _note_forwarded_plugin_command(line)
            return

        if first_line.upper().startswith("REDPITAYA"):
            logger.info("Plugin handshake auto-detected (REDPITAYA)")
            await handle_plugin_handshake(
                reader,
                writer,
                source,
                first_frame_timeout,
                verbose,
                verbose_frames,
                num_ch,
                initial_peek=peek,
            )
            return

        logger.info(
            "Streaming binary for Open Ephys (%d byte client peek)",
            len(peek),
        )
        await stream_frames(
            writer, source, first_frame_timeout, verbose, verbose_frames
        )
    except asyncio.TimeoutError:
        logger.warning("client idle timeout")
    except ConnectionResetError:
        pass
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass
        logger.info("TCP client disconnected")


async def relay_plugin_commands(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    source: SerialFrameSource,
) -> None:
    """Forward FREQ/CFG/FILTER lines from Plugin to USB serial during acquisition."""
    try:
        while True:
            try:
                line = await asyncio.wait_for(read_line(reader), timeout=0.25)
            except asyncio.TimeoutError:
                continue
            if not line:
                break
            upper = line.upper()
            if upper.startswith("REDPITAYA") or upper.startswith("START"):
                continue
            if _should_relay_plugin_command(line):
                logger.info("Plugin → serial: %s", line)
                if upper.startswith("REC "):
                    await relay_rec_command(writer, source, line)
                else:
                    forward_serial_command(source, line)
                    _note_forwarded_plugin_command(line)
    except (asyncio.CancelledError, ConnectionResetError):
        pass


async def stream_frames(
    writer: asyncio.StreamWriter,
    source: SerialFrameSource,
    first_frame_timeout: float,
    verbose: bool = False,
    verbose_frames: int = 5,
    command_reader: asyncio.StreamReader | None = None,
) -> None:
    """Forward serial frames until the TCP peer closes (Ephys Socket is read-only)."""
    loop = asyncio.get_event_loop()
    first_frame = True
    warned_no_frames = False
    sent = 0
    relay_task = None
    if command_reader is not None:
        relay_task = asyncio.create_task(relay_plugin_commands(command_reader, writer, source))
    try:
        while True:
            timeout = first_frame_timeout if first_frame else 1.0
            frame = await loop.run_in_executor(None, source.get_frame, timeout)
            if frame is None:
                if first_frame and not warned_no_frames:
                    warned_no_frames = True
                    hint = source.diagnostic_hint()
                    logger.warning("no serial frames yet — %s", hint or "unknown cause")
                continue
            frame = normalize_frame_for_oe(frame)
            if len(frame) < HEADER_SIZE:
                continue
            hdr = HEADER.unpack_from(frame, 0)
            if not is_valid_header(hdr):
                logger.warning("dropping serial frame with invalid header: %s", header_fields(frame))
                continue
            expected = payload_bytes_from_header(hdr)
            if expected is None or len(frame) != HEADER_SIZE + expected:
                logger.warning(
                    "dropping serial frame with length %d (expected %d): %s",
                    len(frame),
                    HEADER_SIZE + expected,
                    header_fields(frame),
                )
                continue
            if first_frame:
                logger.info(
                    "first Open Ephys frame from serial (%d bytes) — %s",
                    len(frame),
                    header_fields(frame),
                )
            elif verbose and sent < verbose_frames:
                logger.info("frame %d: %s", sent + 1, header_fields(frame))
            first_frame = False
            writer.write(frame)
            await writer.drain()
            sent += 1
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
        pass
    finally:
        if relay_task is not None:
            relay_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await relay_task
    if sent > 1:
        logger.info("stream ended after %d frames", sent)


async def run_server(
    bind: str,
    port: int,
    source: SerialFrameSource,
    first_frame_timeout: float,
    num_ch: int,
    plugin_mode: bool,
    verbose: bool,
    verbose_frames: int,
) -> None:
    server = await asyncio.start_server(
        lambda r, w: handle_client(
            r,
            w,
            source,
            first_frame_timeout,
            num_ch,
            plugin_mode,
            verbose,
            verbose_frames,
        ),
        bind,
        port,
    )
    addrs = ", ".join(str(s.getsockname()) for s in server.sockets or [])
    logger.info("Serial→TCP bridge listening on %s (Open Ephys → %s:%d)", addrs, bind, port)
    async with server:
        await server.serve_forever()


def main() -> None:
    p = argparse.ArgumentParser(
        description="Bridge STEP USB serial (Open Ephys binary) to TCP localhost:5000"
    )
    p.add_argument(
        "port",
        nargs="?",
        default=os.environ.get("SERIAL_PORT"),
        help="Serial port (e.g. COM5 on Windows, /dev/ttyACM0 on Linux)",
    )
    p.add_argument("--baud", type=int, default=int(os.environ.get("SERIAL_BAUD", "115200")))
    p.add_argument("--bind", default=os.environ.get("BRIDGE_BIND", "127.0.0.1"))
    p.add_argument("--tcp-port", type=int, default=int(os.environ.get("BRIDGE_PORT", "5000")))
    p.add_argument(
        "--plugin",
        action="store_true",
        help="Plugin AcqBoard mode: wait for REDPITAYA/START, emit STARTED/SENSORS, then stream",
    )
    p.add_argument(
        "--csv",
        action="store_true",
        help="Parse CSV serial (SERIAL_OUTPUT_BINARY false) instead of binary",
    )
    p.add_argument(
        "--first-frame-timeout",
        type=float,
        default=float(os.environ.get("BRIDGE_FIRST_FRAME_TIMEOUT", DEFAULT_FIRST_FRAME_TIMEOUT)),
        help="Seconds to wait for the first serial frame (covers 5 s boot delay)",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Log Open Ephys header fields for the first N TCP frames",
    )
    p.add_argument(
        "--verbose-frames",
        type=int,
        default=5,
        help="Number of frames to log with --verbose (includes the first frame)",
    )
    args = p.parse_args()

    if not args.port:
        print("Usage: python host/serial_tcp_bridge.py COM5", file=sys.stderr)
        sys.exit(1)

    num_ch = int(os.environ.get("ESP32_NUM_CHANNELS", str(DEFAULT_NUM_CHANNELS)))

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    payload = "CSV" if args.csv else "Open Ephys binary"
    client_mode = "Plugin AcqBoard (--plugin)" if args.plugin else "Ephys Socket or auto Plugin"
    print(
        f"Serial {args.port} @ {args.baud} → TCP {args.bind}:{args.tcp_port} "
        f"({payload}; {num_ch} ch; {client_mode})"
    )
    print(
        "Close Serial Monitor before starting. "
        "Flash with USB_OPEN_EPHYS_MODE true; wait >5 s after boot."
    )
    if args.plugin:
        print("Open Ephys Plugin AcqBoard: Node IP 127.0.0.1 (port 5000)")
    else:
        print("Open Ephys Ephys Socket → 127.0.0.1:5000 (or use --plugin for AcqBoard)")

    source = SerialFrameSource(args.port, args.baud, csv_mode=args.csv)
    source.start()
    if not args.csv:
        logger.info(
            "Waiting up to %.0fs for first serial frame before TCP listen…",
            args.first_frame_timeout,
        )
        if source.wait_for_frames(args.first_frame_timeout):
            logger.info("Serial frame sync OK — starting TCP server")
        else:
            hint = source.diagnostic_hint()
            logger.warning(
                "No frames before TCP listen — %s",
                hint or "Open Ephys may connect before stream starts",
            )
    try:
        asyncio.run(
            run_server(
                args.bind,
                args.tcp_port,
                source,
                args.first_frame_timeout,
                num_ch,
                args.plugin,
                args.verbose,
                args.verbose_frames,
            )
        )
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        source.stop()


if __name__ == "__main__":
    main()
