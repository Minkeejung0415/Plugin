"""
Curated joint catalog for OpenSim Live HUD display filtering.
Pure Python — no OpenSim import (Python 3.8 unit-test friendly).

This module is the Python mirror of opensim_joint_catalog.h.
Keep both files in sync when adding or renaming joints.

The catalog maps between three representations of the same joint:
  coordinate  — full OpenSim name used in model.getCoordinateSet().get()
  abbrev      — short label shown on the HUD (fits in the viewport)
  segment     — PhysicalOffsetFrame name in Rajagopal2015_opensense_calibrated.osim

Typical caller flow in opensim_live_realtime.py:
  names = get_display_filter_joints()          # list of coordinate names
  for name in names:
      angle = _read_coord_value(model, state, name)  # calls coordinate_for internally
      line  = format_angle_line(abbrev_for(name), angle)
"""

# Hard cap on how many joints can appear in the HUD at once (screen space).
MAX_DISPLAY_JOINTS = 6

# Shown when no explicit selection has been made yet.
DEFAULT_DISPLAY_JOINT = "knee_angle_r"

# Master list — same order as opensim_joint_catalog.h (proximal → distal).
JOINT_CATALOG = [
    {"coordinate": "pelvis_tilt",   "abbrev": "pelvis_tilt", "segment": "pelvis_imu"},    # pelvis forward tilt
    {"coordinate": "hip_flexion_r", "abbrev": "hip_r",       "segment": "femur_r_imu"},   # right hip flex
    {"coordinate": "knee_angle_r",  "abbrev": "knee_r",      "segment": "tibia_r_imu"},   # right knee
    {"coordinate": "ankle_angle_r", "abbrev": "ankle_r",     "segment": "calcn_r_imu"},   # right ankle
    {"coordinate": "hip_flexion_l", "abbrev": "hip_l",       "segment": "femur_l_imu"},   # left hip flex
    {"coordinate": "knee_angle_l",  "abbrev": "knee_l",      "segment": "tibia_l_imu"},   # left knee
    {"coordinate": "ankle_angle_l", "abbrev": "ankle_l",     "segment": "calcn_l_imu"},   # left ankle
]

# Lookup tables built once at import time for fast access.
_CATALOG_BY_COORD = {entry["coordinate"]: entry for entry in JOINT_CATALOG}  # coord → full entry
_CATALOG_ORDER    = [entry["coordinate"] for entry in JOINT_CATALOG]          # ordered coord list
_ABBREV_TO_COORD  = {entry["abbrev"]: entry["coordinate"] for entry in JOINT_CATALOG}  # abbrev → coord


def abbrev_for(coordinate: str) -> str:
    """Return the short HUD label for a coordinate name or abbreviation.

    Example: abbrev_for("knee_angle_r") → "knee_r"
             abbrev_for("knee_r")       → "knee_r"   (already an abbrev)
             abbrev_for("unknown")      → "unknown"  (passthrough)
    """
    entry = _CATALOG_BY_COORD.get(coordinate_for(coordinate))
    if entry is None:
        return coordinate  # unknown name: return as-is rather than crashing
    return entry["abbrev"]


def coordinate_for(name: str) -> str:
    """Resolve a HUD abbreviation OR full coordinate name to the canonical OpenSim name.

    Needed because opensim_joint_display_config.json may contain either form.
    Example: coordinate_for("knee_r")      → "knee_angle_r"
             coordinate_for("knee_angle_r") → "knee_angle_r"  (already canonical)
             coordinate_for("unknown")      → "unknown"        (passthrough)
    """
    if name in _CATALOG_BY_COORD:
        return name  # already a full coordinate name
    return _ABBREV_TO_COORD.get(name, name)  # try abbrev lookup, else passthrough


def validate_joints(joints):
    """Filter a list of joint names to only catalog-known entries, in catalog order.

    - Strips duplicates (first occurrence wins).
    - Ignores unknown names silently.
    - Truncates to MAX_DISPLAY_JOINTS.
    - Resolves abbreviations to full coordinate names.

    Example: validate_joints(["knee_r", "knee_angle_r", "hip_flexion_r", "bad_name"])
             → ["hip_flexion_r", "knee_angle_r"]
             (deduped, bad name dropped, sorted by catalog order)
    """
    if not joints:
        return []
    seen = set()
    for name in joints:
        coord = coordinate_for(name)
        if coord not in _CATALOG_BY_COORD or coord in seen:
            continue  # unknown or duplicate
        seen.add(coord)
    # Re-sort into catalog order so the HUD is always proximal → distal.
    ordered = [name for name in _CATALOG_ORDER if name in seen]
    return ordered[:MAX_DISPLAY_JOINTS]


def format_angle_line(abbrev: str, degrees: float) -> str:
    """Format a single HUD line, e.g. 'knee_r: 35.2°'."""
    return f"{abbrev}: {degrees:.1f}°"
