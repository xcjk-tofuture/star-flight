#ifndef UAV_GYRO_CALIBRATION_H
#define UAV_GYRO_CALIBRATION_H
#include <stdint.h>

typedef struct {
    uint16_t samples;
    float gravity_m_s2, acceleration_tolerance_m_s2;
    float max_rate_rad_s, max_rate_std_rad_s, max_acceleration_std_m_s2;
} uav_gyro_calibration_config_t;
typedef struct {
    uav_gyro_calibration_config_t config;
    uint16_t count;
    uint8_t ready, failed, reject_reason;
    float mean[6], m2[6];
} uav_gyro_calibration_t;
enum { UAV_GYRO_COLLECTING, UAV_GYRO_REJECTED, UAV_GYRO_READY, UAV_GYRO_FAILED };
enum { UAV_GYRO_REASON_NONE, UAV_GYRO_REASON_INVALID, UAV_GYRO_REASON_RATE,
       UAV_GYRO_REASON_GRAVITY, UAV_GYRO_REASON_VARIANCE, UAV_GYRO_REASON_COUNT };

/* Caller owns context; no allocation/blocking/RTOS. Invalid config returns -1.
 * Feed fresh, settled low-pass gyro rad/s and acceleration m/s^2; gyro must
 * include its existing offset so the returned bias is an absolute offset.
 * Motion/nonfinite data resets the entire contiguous window. Bias is written
 * only on READY and remains latched until init; no temperature compensation. */
int uav_gyro_calibration_init(uav_gyro_calibration_t *state,
                              const uav_gyro_calibration_config_t *config);
int uav_gyro_calibration_feed(uav_gyro_calibration_t *state, const float gyro[3],
                              const float acceleration[3], float bias[3]);
/* Caller supplies wrap-safe elapsed time. Expiry latches failure until init;
 * never marks an unfinished calibration READY or overwrites the caller bias. */
int uav_gyro_calibration_expire(uav_gyro_calibration_t *state, uint32_t elapsed_ms,
                              uint32_t timeout_ms);
#endif
