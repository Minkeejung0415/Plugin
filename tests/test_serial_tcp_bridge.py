import importlib.util
import struct
from pathlib import Path


def load_bridge_module():
    path = Path(__file__).resolve().parents[1] / "esp32" / "host" / "serial_tcp_bridge.py"
    spec = importlib.util.spec_from_file_location("serial_tcp_bridge", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pack_csv_row_preserves_14_channel_magnetometer_slots():
    bridge = load_bridge_module()
    fields = ["42"] + [str(i) for i in range(14)]

    frame = bridge.pack_csv_row(fields, num_channels=14)

    assert frame is not None
    hdr = bridge.HEADER.unpack_from(frame, 0)
    assert hdr == (0, 28, bridge.OE_BIT_DEPTH_S16, 2, 14, 1)
    channels = struct.unpack_from("<14h", frame, bridge.HEADER_SIZE)
    assert channels[6:9] == (6, 7, 8)
    assert channels[13] == 13


def test_pack_csv_row_keeps_legacy_8_channel_override():
    bridge = load_bridge_module()
    fields = ["42"] + [str(i) for i in range(14)]

    frame = bridge.pack_csv_row(fields, num_channels=8)

    assert frame is not None
    hdr = bridge.HEADER.unpack_from(frame, 0)
    assert hdr == (0, 16, bridge.OE_BIT_DEPTH_S16, 2, 8, 1)
    channels = struct.unpack_from("<8h", frame, bridge.HEADER_SIZE)
    assert channels == tuple(range(8))


if __name__ == "__main__":
    test_pack_csv_row_preserves_14_channel_magnetometer_slots()
    test_pack_csv_row_keeps_legacy_8_channel_override()
