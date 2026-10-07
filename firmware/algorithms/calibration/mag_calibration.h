#ifndef UAV_MAG_CALIBRATION_H
#define UAV_MAG_CALIBRATION_H
#include <stdint.h>
#define UAV_MAG_CAL_CAPACITY 512u
#define UAV_MAG_CAL_MIN_SAMPLES 200u
#define UAV_MAG_CAL_TIMEOUT_MS 180000u
enum { UAV_MAG_CAL_OK=0, UAV_MAG_CAL_SAMPLES=1, UAV_MAG_CAL_SPAN=2,
       UAV_MAG_CAL_SINGULAR=3, UAV_MAG_CAL_SHAPE=4, UAV_MAG_CAL_COVERAGE=5,
       UAV_MAG_CAL_RESIDUAL=6, UAV_MAG_CAL_STORAGE=7, UAV_MAG_CAL_TIMEOUT=8 };
typedef struct {
    float bias[3], matrix[9]; /* corrected_uT = matrix * (raw_uT - bias) */
    float field_ut, rms_fraction;
    uint32_t coverage, samples;
} uav_mag_calibration_t;
typedef struct {
    float points[UAV_MAG_CAL_CAPACITY][3], last[3];
    double solve[9][10]; /* Caller-owned scratch; no large task-stack allocation. */
    uint32_t seen, rng, last_ms;
    uint16_t count;
    uint8_t have_last;
} uav_mag_calibrator_t;
void uav_mag_calibrator_init(uav_mag_calibrator_t *cal);
int uav_mag_calibrator_feed(uav_mag_calibrator_t *cal, const float raw[3], uint32_t now_ms);
int uav_mag_calibrator_fit(uav_mag_calibrator_t *cal, uav_mag_calibration_t *result);
int uav_mag_calibration_valid(const uav_mag_calibration_t *cal);
void uav_mag_correct(const float matrix[9], const float bias[3], const float raw[3], float out[3]);
#endif
