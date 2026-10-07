#ifndef UAV_IMU_CALIBRATION_CONFIG_H
#define UAV_IMU_CALIBRATION_CONFIG_H
#include "gyro_calibration.h"
/* 500 fresh 5 ms samples. Provisional bounds require a no-prop bench check. */
static const uav_gyro_calibration_config_t uav_board_gyro_calibration = {
    .samples = 500,
    .gravity_m_s2 = 9.80665f,
    .acceleration_tolerance_m_s2 = 0.8f,
    .max_rate_rad_s = 0.15f,
    .max_rate_std_rad_s = 0.005f,
    .max_acceleration_std_m_s2 = 0.15f
};
#endif
