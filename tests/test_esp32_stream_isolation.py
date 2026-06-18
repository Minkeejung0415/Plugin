"""Focused tests for SD-authoritative, lossy-stream stress verdicts."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "esp32" / "host" / "stress_test_serial.py"
)
SPEC = importlib.util.spec_from_file_location("stress_test_serial", MODULE_PATH)
assert SPEC and SPEC.loader
stress = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = stress
SPEC.loader.exec_module(stress)


def evaluate(**overrides):
    values = {
        "sd_on": True,
        "stream_on": True,
        "hz": 1000,
        "capture_elapsed": 60.0,
        "expected": 51000,
        "sd_final_seen": True,
        "sd_saved": 59000,
        "sd_errors": 0,
        "loop_overruns": 0,
        "rows": 40000,
        "stream_mean_hz": 850.0,
        "dup": 12,
        "gap": 5000,
    }
    values.update(overrides)
    return stress.evaluate_paths(**values)


def test_udp_degradation_does_not_fail_clean_sd():
    sd_hz, sd_pass, stream_quality, overall = evaluate()
    assert 983.0 < sd_hz < 984.0
    assert sd_pass is True
    assert stream_quality == "DEGRADED"
    assert overall is True


def test_sd_error_fails_even_when_stream_is_clean():
    _, sd_pass, stream_quality, overall = evaluate(
        sd_errors=1,
        rows=60000,
        stream_mean_hz=1000.0,
        dup=0,
        gap=0,
    )
    assert sd_pass is False
    assert stream_quality == "PASS"
    assert overall is False


def test_sd_rate_below_threshold_fails():
    _, sd_pass, _, overall = evaluate(sd_saved=54000)
    assert sd_pass is False
    assert overall is False


def test_stream_only_keeps_strict_transport_verdict():
    _, sd_pass, stream_quality, overall = evaluate(
        sd_on=False,
        sd_final_seen=False,
        sd_saved=None,
        sd_errors=None,
    )
    assert sd_pass is None
    assert stream_quality == "DEGRADED"
    assert overall is False


def test_stream_off_uses_sd_only():
    _, sd_pass, stream_quality, overall = evaluate(
        stream_on=False,
        rows=0,
        stream_mean_hz=None,
        dup=0,
        gap=0,
    )
    assert sd_pass is True
    assert stream_quality == "OFF"
    assert overall is True


def test_stream_counters_parse_from_status():
    values = stress.parse_key_value_parts(
        "stream_offered=60000 stream_sent=58000 "
        "stream_queue_drops=1990 stream_send_errors=10"
    )
    assert stress.maybe_int(values, "stream_offered") == 60000
    assert stress.maybe_int(values, "stream_sent") == 58000
    assert stress.maybe_int(values, "stream_queue_drops") == 1990
    assert stress.maybe_int(values, "stream_send_errors") == 10


if __name__ == "__main__":
    tests = [
        value
        for name, value in sorted(globals().items())
        if name.startswith("test_") and callable(value)
    ]
    for test in tests:
        test()
    print(f"{len(tests)} stream-isolation tests passed")
