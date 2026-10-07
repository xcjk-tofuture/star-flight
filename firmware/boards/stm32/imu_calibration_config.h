#ifndef UAV_IMU_CALIBRATION_CONFIG_H
#define UAV_IMU_CALIBRATION_CONFIG_H
#include "gyro_calibration.h"
/* Dedicated 5 Hz calibration path, 500 ms settling then 500 fusion samples.
 * Control gyro has its separate 40 Hz bandwidth. No raw spikes enter this window. */
#define UAV_GYRO_CAL_WARMUP_MS 500u
#define UAV_GYRO_CAL_TIMEOUT_MS 30000u
static const uav_gyro_calibration_config_t uav_board_gyro_calibration = {
    .samples = 500,
    .gravity_m_s2 = 9.80665f,
    .acceleration_tolerance_m_s2 = 0.8f,
    .max_rate_rad_s = 0.15f,
    .max_rate_std_rad_s = 0.015f,
    .max_acceleration_std_m_s2 = 0.15f
};
#endif
