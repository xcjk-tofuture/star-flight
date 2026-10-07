#ifndef UAV_FUSION_H
#define UAV_FUSION_H
#include <stdint.h>
#include "biquad.h"
typedef struct {
    float gravity_m_s2, accel_gain, mag_gain, bias_gain, bias_limit_rad_s;
    float mag_min_ut, mag_max_ut, mag_relative_tolerance, mag_innovation_rad;
    float mag_sample_hz, mag_cutoff_hz;
    uint32_t mag_timeout_us;
    uint8_t mag_recovery_samples;
} uav_fusion_config_t;
typedef struct {
    uav_fusion_config_t config;
    float q[4], gyro_bias[3], roll_deg, pitch_deg, yaw_deg;
    float accel_weight, mag_weight, mag_reference_ut, mag_innovation_rad;
    uav_biquad_t mag_filter;
    float mag_earth[3], mag_norm_ut, mag_horizontal_ut;
    uint32_t accel_rejected, mag_rejected, invalid_updates, last_mag_us;
    uint8_t initialized, mag_used, mag_good_samples, have_mag_timestamp, mag_reason;
} uav_fusion_t;
enum { UAV_FUSION_MAG_AVAILABLE=1, UAV_FUSION_ALLOW_MAG_RECOVERY=2 };
enum { UAV_MAG_OK, UAV_MAG_MISSING, UAV_MAG_STALE, UAV_MAG_RANGE,
       UAV_MAG_FIELD_CHANGE, UAV_MAG_VERTICAL, UAV_MAG_INNOVATION, UAV_MAG_RECOVERING };
int uav_fusion_init(uav_fusion_t *fusion, const uav_fusion_config_t *config);
/* Body-to-earth quaternion. Legacy Euler adapter preserves this project's
 * roll/+ and pitch,yaw/- convention. Gyro uses SI rad/s; real dt seconds.
 * Magnetic failures degrade to inertial fusion; invalid gyro/dt returns -1.
 * mag_available is a bitmask. Large heading recovery requires the explicit
 * disarmed permission bit and stationary gyro/gravity, then fresh sample streak. */
int uav_fusion_step(uav_fusion_t *fusion, const float acc[3], const float gyro[3],
                    const float mag[3], uint32_t now_us, uint32_t mag_timestamp_us,
                    int mag_available, float dt_s);
#endif
