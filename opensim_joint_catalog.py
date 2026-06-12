"""
Curated joint catalog for OpenSim Live HUD display filtering.
Pure Python - no OpenSim import (Python 3.8 unit-test friendly).

This module is the Python mirror of opensim_joint_catalog.h.
Keep both files in sync when adding or renaming joints.

The catalog maps between three representations of the same joint:
  coordinate - full OpenSim name used in model.getCoordinateSet().get()
  abbrev     - short label shown on the HUD
  segment    - PhysicalOffsetFrame name in Rajagopal2015_opensense_calibrated.osim
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
_ABBREV_TO_COORD = {entry["abbrev"]: entry["coordinate"] for entry in JOINT_CATALOG}


def abbrev_for(coordinate: str) -> str:
    """Return the short HUD label for a coordinate name or abbreviation."""
    entry = _CATALOG_BY_COORD.get(coordinate_for(coordinate))
    if entry is None:
        return coordinate
    return entry["abbrev"]


def coordinate_for(name: str) -> str:
    """Resolve a HUD abbreviation or full coordinate name to the canonical name."""
    if name in _CATALOG_BY_COORD:
        return name
    return _ABBREV_TO_COORD.get(name, name)


def validate_joints(joints):
    """Filter joint names to catalog-known entries while preserving input order."""
    if not joints:
        return []

    seen = set()
    validated = []
    for name in joints:
        coord = coordinate_for(name)
        if coord not in _CATALOG_BY_COORD or coord in seen:
            continue
        seen.add(coord)
        validated.append(coord)

    return validated[:MAX_DISPLAY_JOINTS]


def format_angle_line(abbrev: str, degrees: float) -> str:
    return f"{abbrev}: {degrees:.1f} deg"
