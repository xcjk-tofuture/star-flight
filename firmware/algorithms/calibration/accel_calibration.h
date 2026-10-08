#ifndef UAV_ACCEL_CALIBRATION_H
#define UAV_ACCEL_CALIBRATION_H
#include <stdint.h>
#define UAV_ACCEL_CAL_GRAVITY 9.80665f
#define UAV_ACCEL_CAL_SAMPLES 200u
#define UAV_ACCEL_CAL_TIMEOUT_MS 300000u
enum { UAV_ACCEL_PLACE=0, UAV_ACCEL_SETTLE, UAV_ACCEL_SAMPLE, UAV_ACCEL_FIT,
       UAV_ACCEL_SAVE, UAV_ACCEL_DONE, UAV_ACCEL_FAILED };
/* Face order: Z+, Z-, X+, X-, Y+, Y-. SI/body axes, not installation rotation. */
enum { UAV_ACCEL_REASON_OK=0, UAV_ACCEL_REASON_CONFIRM, UAV_ACCEL_REASON_ORIENTATION,
       UAV_ACCEL_REASON_MOTION, UAV_ACCEL_REASON_NOISE, UAV_ACCEL_REASON_TIMING,
       UAV_ACCEL_REASON_FIT, UAV_ACCEL_REASON_STORAGE, UAV_ACCEL_REASON_TIMEOUT };
typedef struct {
    float bias[3], scale[3], gravity_m_s2, rms_fraction;
    uint32_t faces, samples;
} uav_accel_calibration_t;
typedef struct {
    float faces[6][3], mean[3], m2[3], raw_mean[3], raw_m2[3];
    uint32_t last_ms, stable_ms, noisy_ms;
    uint16_t count;
    uint8_t mask, target, detected, phase, reason, have_time, stable, noise_latched;
} uav_accel_calibrator_t;
void uav_accel_calibrator_init(uav_accel_calibrator_t *cal);
void uav_accel_calibrator_confirm(uav_accel_calibrator_t *cal);
void uav_accel_calibrator_discard_window(uav_accel_calibrator_t *cal);
/* Feed at 100 Hz. Uncorrected 5 Hz accel for mean/pose, raw accel for noise,
 * bias-corrected gyro for movement. Returns 1 when all six faces are ready. */
int uav_accel_calibrator_feed(uav_accel_calibrator_t *cal, const float raw[3],
                             const float filtered[3], const float gyro[3], uint32_t now_ms);
/* Returns OK, or FIT and removes the worst tilted face for recapture. */
int uav_accel_calibrator_fit(uav_accel_calibrator_t *cal, uav_accel_calibration_t *out);
int uav_accel_calibration_valid(const uav_accel_calibration_t *cal);
#endif
