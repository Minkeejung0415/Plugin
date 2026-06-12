#pragma once

#include <stdint.h>

void camera_verify_init(void);
int16_t camera_verify_motion_score(void);
int16_t camera_verify_action_flag(int16_t imu_motion_metric);
