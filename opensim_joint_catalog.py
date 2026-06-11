"""
Curated joint catalog for OpenSim Live HUD display filtering.
Pure Python — no OpenSim import (Python 3.8 unit-test friendly).
"""

MAX_DISPLAY_JOINTS = 6
DEFAULT_DISPLAY_JOINT = "knee_angle_r"

JOINT_CATALOG = [
    {"coordinate": "pelvis_tilt", "abbrev": "pelvis_tilt", "segment": "pelvis_imu"},
    {"coordinate": "hip_flexion_r", "abbrev": "hip_r", "segment": "femur_r_imu"},
    {"coordinate": "knee_angle_r", "abbrev": "knee_r", "segment": "tibia_r_imu"},
    {"coordinate": "ankle_angle_r", "abbrev": "ankle_r", "segment": "calcn_r_imu"},
    {"coordinate": "hip_flexion_l", "abbrev": "hip_l", "segment": "femur_l_imu"},
    {"coordinate": "knee_angle_l", "abbrev": "knee_l", "segment": "tibia_l_imu"},
    {"coordinate": "ankle_angle_l", "abbrev": "ankle_l", "segment": "calcn_l_imu"},
]

_CATALOG_BY_COORD = {entry["coordinate"]: entry for entry in JOINT_CATALOG}
_CATALOG_ORDER = [entry["coordinate"] for entry in JOINT_CATALOG]
_ABBREV_TO_COORD = {entry["abbrev"]: entry["coordinate"] for entry in JOINT_CATALOG}


def abbrev_for(coordinate: str) -> str:
    entry = _CATALOG_BY_COORD.get(coordinate_for(coordinate))
    if entry is None:
        return coordinate
    return entry["abbrev"]


def coordinate_for(name: str) -> str:
    """Map HUD abbrev or coordinate string to OpenSim coordinate name."""
    if name in _CATALOG_BY_COORD:
        return name
    return _ABBREV_TO_COORD.get(name, name)


def validate_joints(joints):
    """Return catalog-known coordinates in catalog order, at most MAX_DISPLAY_JOINTS."""
    if not joints:
        return []
    seen = set()
    for name in joints:
        coord = coordinate_for(name)
        if coord not in _CATALOG_BY_COORD or coord in seen:
            continue
        seen.add(coord)
    ordered = [name for name in _CATALOG_ORDER if name in seen]
    return ordered[:MAX_DISPLAY_JOINTS]


def format_angle_line(abbrev: str, degrees: float) -> str:
    return f"{abbrev}: {degrees:.1f}°"
