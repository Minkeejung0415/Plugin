/*
    Curated joint coordinate catalog for OpenSim Live HUD display.
    Keep in sync with opensim_joint_catalog.py (Python 3.8).

    HOW THIS IS USED:
    - The C++ plugin (device editor.cpp) iterates kOpenSimJointCatalog to
      build the joint-selection checkboxes in the UI.
    - AcqBoardRedPitaya uses kOpenSimJointCatalog to look up entry details
      when writing opensim_joint_display_config.json.
    - The Python bridge (opensim_live_realtime.py) reads the JSON and
      calls model.getCoordinateSet().get(coordinate) for each selected joint.

    FIELDS:
      coordinate → the exact name used in the .osim model's <CoordinateSet>
      abbrev     → short label shown on the HUD (fits in the viewport)
      segment    → the PhysicalOffsetFrame name the IMU sensor attaches to
                   in Rajagopal2015_opensense_calibrated.osim

    ADDING A JOINT:
      1. Add a row here.
      2. Add the matching row to JOINT_CATALOG in opensim_joint_catalog.py.
      3. Increment kOpenSimJointCatalogSize.
      4. Make sure the coordinate name exists in the .osim model file.
*/

#pragma once

/* One row in the joint catalog. */
struct OpenSimJointCatalogEntry
{
    const char* coordinate; /* Full OpenSim coordinate name (used in getCoordinateSet().get()) */
    const char* abbrev;     /* Short display name shown on the HUD, e.g. "knee_r" */
    const char* segment;    /* IMU frame name in the .osim model, e.g. "tibia_r_imu" */
};

/* Total number of joints in the catalog (must equal the array size below). */
static constexpr int kOpenSimJointCatalogSize = 7;

/* The HUD will show at most this many joints simultaneously (UI/space limit). */
static constexpr int kMaxJointDisplaySelection = 6;

/* Index into kOpenSimJointCatalog used when no selection has been made yet.
 * Index 2 = "knee_angle_r" — a sensible clinical default.                 */
static constexpr int kDefaultJointDisplayCatalogIndex = 2;

/* The coordinate name corresponding to the default index above. */
static constexpr const char* kDefaultJointDisplayCoordinate = "knee_angle_r";

/* Master list of displayable joints.  Order here controls display order in
 * the HUD (proximal → distal on each side).                                */
static constexpr OpenSimJointCatalogEntry kOpenSimJointCatalog[kOpenSimJointCatalogSize] = {
    /*  coordinate         abbrev      segment          */
    { "pelvis_tilt",   "pelvis", "pelvis_imu"   },  /* forward/backward tilt of the pelvis */
    { "hip_flexion_r", "hip_r",  "femur_r_imu"  },  /* right hip bend (thigh forward/back) */
    { "knee_angle_r",  "knee_r", "tibia_r_imu"  },  /* right knee (shin vs thigh angle)    */
    { "ankle_angle_r", "ankle_r","calcn_r_imu"  },  /* right ankle (foot plantarflexion)   */
    { "hip_flexion_l", "hip_l",  "femur_l_imu"  },  /* left hip bend                       */
    { "knee_angle_l",  "knee_l", "tibia_l_imu"  },  /* left knee                           */
    { "ankle_angle_l", "ankle_l","calcn_l_imu"  },  /* left ankle                          */
};
