#ifndef UAV_FUSION_H
#define UAV_FUSION_H
#include <stdint.h>
typedef struct {
    float gravity_m_s2, accel_gain, mag_gain, bias_gain, bias_limit_rad_s;
    float mag_min_ut, mag_max_ut, mag_relative_tolerance, mag_innovation_rad;
    uint32_t mag_timeout_us;
    uint8_t mag_recovery_samples;
} uav_fusion_config_t;
typedef struct {
    uav_fusion_config_t config;
    float q[4], gyro_bias[3], roll_deg, pitch_deg, yaw_deg;
    float accel_weight, mag_weight, mag_reference_ut, mag_innovation_rad;
    uint32_t accel_rejected, mag_rejected, invalid_updates, last_mag_us;
    uint8_t initialized, mag_used, mag_good_samples, have_mag_timestamp;
} uav_fusion_t;
int uav_fusion_init(uav_fusion_t *fusion, const uav_fusion_config_t *config);
/* Body-to-earth quaternion. Legacy Euler adapter preserves this project's
 * roll/+ and pitch,yaw/- convention. Gyro uses SI rad/s; real dt seconds.
 * Magnetic failures degrade to inertial fusion; invalid gyro/dt returns -1. */
int uav_fusion_step(uav_fusion_t *fusion, const float acc[3], const float gyro[3],
                    const float mag[3], uint32_t now_us, uint32_t mag_timestamp_us,
                    int mag_available, float dt_s);
#endif
