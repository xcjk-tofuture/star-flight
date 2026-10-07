#ifndef UAV_IMU_PROCESSING_CONFIG_H
#define UAV_IMU_PROCESSING_CONFIG_H
#include "imu_pipeline.h"
#include "fusion.h"
#define UAV_IMU_SAMPLE_PERIOD_MS 2u
#define UAV_IMU_FUSION_PERIOD_US 5000u
#define UAV_IMU_MAG_PERIOD_MS 20u
#define UAV_IMU_BARO_PERIOD_MS 50u
#define UAV_IMU_TEMPERATURE_PERIOD_MS 100u
/* Application-owned register profile, applied after the unchanged self tests.
 * Accel 800 Hz normal bandwidth; gyro 1000 Hz / 116 Hz bandwidth. */
#define UAV_IMU_ACCEL_CONFIG 0xabu
#define UAV_IMU_GYRO_BANDWIDTH 0x02u
static const uav_imu_config_t uav_board_imu_processing={500,40,30,5,10000};
static const uav_fusion_config_t uav_board_fusion={
    .gravity_m_s2=9.80665f,.accel_gain=3,.mag_gain=1,.bias_gain=.05f,
    .bias_limit_rad_s=.05f,.mag_min_ut=10,.mag_max_ut=100,
    .mag_relative_tolerance=.25f,.mag_innovation_rad=.785398163f,
    .mag_timeout_us=100000,.mag_recovery_samples=10
};
#endif
