#include "gyro_calibration.h"
#include <math.h>
#include <string.h>

static void reset_window(uav_gyro_calibration_t *s, uint8_t reason) {
    s->count = 0;
    s->reject_reason = reason;
    memset(s->mean, 0, sizeof s->mean);
    memset(s->m2, 0, sizeof s->m2);
}
int uav_gyro_calibration_init(uav_gyro_calibration_t *s,
                              const uav_gyro_calibration_config_t *c) {
    if (!s || !c || c->samples < 2 || !isfinite(c->gravity_m_s2) ||
        !isfinite(c->acceleration_tolerance_m_s2) || !isfinite(c->max_rate_rad_s) ||
        !isfinite(c->max_rate_std_rad_s) || !isfinite(c->max_acceleration_std_m_s2) ||
        c->gravity_m_s2 <= 0 || c->acceleration_tolerance_m_s2 <= 0 ||
        c->acceleration_tolerance_m_s2 >= c->gravity_m_s2 || c->max_rate_rad_s <= 0 ||
        c->max_rate_std_rad_s <= 0 || c->max_acceleration_std_m_s2 <= 0)
        return -1;
    memset(s, 0, sizeof *s);
    s->config = *c;
    return 0;
}
int uav_gyro_calibration_feed(uav_gyro_calibration_t *s, const float g[3],
                              const float a[3], float bias[3]) {
    if (!s || !g || !a || !bias || s->config.samples < 2)
        return UAV_GYRO_REJECTED;
    if (s->failed)
        return UAV_GYRO_FAILED;
    if (s->ready) {
        memcpy(bias, s->mean, 3 * sizeof(float));
        return UAV_GYRO_READY;
    }
    float norm2 = 0;
    for (unsigned i = 0; i < 3; i++) {
        if (!isfinite(g[i]) || !isfinite(a[i])) {
            reset_window(s, UAV_GYRO_REASON_INVALID);
            return UAV_GYRO_REJECTED;
        }
        if (fabsf(g[i]) > s->config.max_rate_rad_s) {
            reset_window(s, UAV_GYRO_REASON_RATE);
            return UAV_GYRO_REJECTED;
        }
        norm2 += a[i] * a[i];
    }
    if (!isfinite(norm2) ||
        fabsf(sqrtf(norm2) - s->config.gravity_m_s2) >
            s->config.acceleration_tolerance_m_s2) {
        reset_window(s, UAV_GYRO_REASON_GRAVITY);
        return UAV_GYRO_REJECTED;
    }
    s->count++;
    for (unsigned i = 0; i < 6; i++) {
        float x = i < 3 ? g[i] : a[i - 3];
        float delta = x - s->mean[i];
        s->mean[i] += delta / s->count;
        s->m2[i] += delta * (x - s->mean[i]);
    }
    if (s->count < s->config.samples)
        return UAV_GYRO_COLLECTING;
    for (unsigned i = 0; i < 6; i++) {
        float limit = i < 3 ? s->config.max_rate_std_rad_s :
                             s->config.max_acceleration_std_m_s2;
        if (s->m2[i] / (s->count - 1) > limit * limit) {
            reset_window(s, UAV_GYRO_REASON_VARIANCE);
            return UAV_GYRO_REJECTED;
        }
    }
    s->ready = 1;
    memcpy(bias, s->mean, 3 * sizeof(float));
    return UAV_GYRO_READY;
}

int uav_gyro_calibration_expire(uav_gyro_calibration_t *s, uint32_t elapsed_ms,
                              uint32_t timeout_ms) {
    if (!s || !timeout_ms || s->ready)
        return 0;
    if (elapsed_ms >= timeout_ms)
        s->failed = 1;
    return s->failed != 0;
}
