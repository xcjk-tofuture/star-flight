#ifndef UAV_IMU_PIPELINE_H
#define UAV_IMU_PIPELINE_H
#include "biquad.h"
#include <stdint.h>
typedef struct {
    float sample_hz, gyro_cutoff_hz, accel_cutoff_hz, calibration_cutoff_hz;
    uint32_t max_gap_us;
} uav_imu_config_t;
typedef struct {
    uint32_t samples, invalid_samples, timing_resets, filter_resets, dt_min_us, dt_max_us;
    float measured_hz;
} uav_imu_stats_t;
typedef struct {
    float acc[3], gyro_control[3], gyro_calibration[3];
    uint32_t timestamp_us;
    uint8_t valid;
} uav_imu_sample_t;
typedef struct {
    float acc[3], gyro_average[3], gyro_control[3], gyro_calibration[3], dt_s;
    uint32_t timestamp_us;
} uav_imu_frame_t;
typedef struct {
    uav_imu_config_t config;
    uav_biquad_t accel_filter, gyro_filter, calibration_filter;
    uav_imu_stats_t stats;
    uav_imu_sample_t sample;
    float gyro_bias[3], previous_gyro[3], angle_integral[3], configured_hz;
    uint32_t integrated_us, previous_us, rate_us;
    uint16_t rate_samples;
    uint8_t have_previous;
} uav_imu_pipeline_t;
int uav_imu_pipeline_init(uav_imu_pipeline_t *pipeline, const uav_imu_config_t *config);
/* Sensor-owner task only. Finite SI vectors, midpoint timestamp in microseconds.
 * Invalid samples/gaps discard pending integration; no invented catch-up data. */
int uav_imu_pipeline_push(uav_imu_pipeline_t *pipeline, const float acc[3],
                          const float gyro[3], uint32_t timestamp_us);
/* Consume all observed gyro intervals since the preceding frame. */
int uav_imu_pipeline_consume(uav_imu_pipeline_t *pipeline, uav_imu_frame_t *frame);
void uav_imu_pipeline_set_bias(uav_imu_pipeline_t *pipeline, const float absolute_bias[3]);
void uav_imu_pipeline_discard(uav_imu_pipeline_t *pipeline);
#endif
