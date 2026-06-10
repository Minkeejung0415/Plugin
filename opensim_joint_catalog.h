/*
    Curated joint coordinate catalog for OpenSim Live HUD display.
    Keep in sync with opensim_joint_catalog.py (Python 3.8).
*/

#pragma once

struct OpenSimJointCatalogEntry
{
    const char* coordinate;
    const char* abbrev;
    const char* segment;
};

static constexpr int kOpenSimJointCatalogSize = 7;
static constexpr int kMaxJointDisplaySelection = 6;

static constexpr OpenSimJointCatalogEntry kOpenSimJointCatalog[kOpenSimJointCatalogSize] = {
    { "pelvis_tilt",   "pelvis", "pelvis_imu"   },
    { "hip_flexion_r", "hip_r",  "femur_r_imu"  },
    { "knee_angle_r",  "knee_r", "tibia_r_imu"  },
    { "ankle_angle_r", "ankle_r","calcn_r_imu"  },
    { "hip_flexion_l", "hip_l",  "femur_l_imu"  },
    { "knee_angle_l",  "knee_l", "tibia_l_imu"  },
    { "ankle_angle_l", "ankle_l","calcn_l_imu"  },
};
