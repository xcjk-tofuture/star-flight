#include "imu_pipeline.h"
#include <math.h>
#include <string.h>
#include <limits.h>
static int configure_filters(uav_imu_pipeline_t *p, float rate) {
    if (uav_biquad_configure(&p->accel_filter, rate, p->config.accel_cutoff_hz) ||
        uav_biquad_configure(&p->gyro_filter, rate, p->config.gyro_cutoff_hz) ||
        uav_biquad_configure(&p->calibration_filter, rate, p->config.calibration_cutoff_hz) ||
        uav_biquad_configure(&p->accel_calibration_filter, rate, p->config.calibration_cutoff_hz))
        return -1;
    p->configured_hz = rate;
    return 0;
}
static void discontinuity(uav_imu_pipeline_t *p) {
    p->have_previous = 0;
    p->integrated_us = 0;
    p->rate_samples = 0;
    p->rate_us = 0;
    memset(p->angle_integral, 0, sizeof(p->angle_integral));
    uav_biquad_reset(&p->accel_filter);
    uav_biquad_reset(&p->gyro_filter);
    uav_biquad_reset(&p->calibration_filter);
    uav_biquad_reset(&p->accel_calibration_filter);
    p->sample.valid = 0;
}
void uav_imu_pipeline_discard(uav_imu_pipeline_t *p) { if (p) discontinuity(p); }
int uav_imu_pipeline_init(uav_imu_pipeline_t *p, const uav_imu_config_t *config) {
    if (!p || !config || !isfinite(config->sample_hz) || config->sample_hz < 100 ||
        config->sample_hz > 2000 || config->max_gap_us < 2000000.0f / config->sample_hz)
        return -1;
    memset(p, 0, sizeof(*p));
    p->config = *config;
    p->stats.dt_min_us = UINT32_MAX;
    p->stats.measured_hz = config->sample_hz;
    for (unsigned i=0;i<3;i++) p->accel_scale[i]=1;
    return configure_filters(p, config->sample_hz);
}
int uav_imu_pipeline_push(uav_imu_pipeline_t *p, const float a[3], const float g[3], uint32_t us) {
    if (!p || !a || !g) return -1;
    for (unsigned i = 0; i < 3; i++) {
        /* BMI088 application profile is +/-3 g and +/-500 deg/s. */
        if (!isfinite(a[i]) || !isfinite(g[i]) || fabsf(a[i]) > 30 || fabsf(g[i]) > 8.6f) {
            p->stats.invalid_samples++;
            discontinuity(p);
            return -1;
        }
    }
    uint32_t dt = (uint32_t)(us - p->previous_us);
    if (p->have_previous && (!dt || dt < 500 || dt > p->config.max_gap_us)) {
        p->stats.timing_resets++;
        discontinuity(p);
        return -1;
    }
    if (p->have_previous) {
        if (dt < p->stats.dt_min_us) p->stats.dt_min_us = dt;
        if (dt > p->stats.dt_max_us) p->stats.dt_max_us = dt;
        p->rate_us += dt;
        p->rate_samples++;
        if (p->rate_samples >= 250) {
            float rate = p->rate_samples * 1000000.0f / p->rate_us;
            p->stats.measured_hz = rate;
            if (fabsf(rate / p->configured_hz - 1) > .05f && rate >= 100 && rate <= 2000) {
                if (configure_filters(p, rate)) { discontinuity(p); return -1; }
                p->stats.filter_resets++;
            }
            p->rate_us = 0; p->rate_samples = 0;
        }
    }
    float filtered_rate[3],corrected[3];
    for (unsigned i=0;i<3;i++) corrected[i]=(a[i]-p->accel_bias[i])*p->accel_scale[i];
    if (uav_biquad_apply(&p->accel_filter, corrected, p->sample.acc) ||
        uav_biquad_apply(&p->accel_calibration_filter, a, p->sample.acc_uncalibrated) ||
        uav_biquad_apply(&p->gyro_filter, g, filtered_rate) ||
        uav_biquad_apply(&p->calibration_filter, g, p->sample.gyro_calibration)) {
        p->stats.invalid_samples++;
        discontinuity(p);
        return -1;
    }
    for (unsigned i = 0; i < 3; i++) {
        p->sample.gyro_control[i] = filtered_rate[i] - p->gyro_bias[i];
        if (p->have_previous)
            p->angle_integral[i] += .5f * (p->previous_gyro[i] + g[i]) * (dt * 1e-6f);
        p->previous_gyro[i] = g[i];
    }
    if (p->have_previous) p->integrated_us += dt;
    p->sample.timestamp_us = p->previous_us = us;
    p->sample.valid = p->have_previous = 1;
    p->stats.samples++;
    return 0;
}
int uav_imu_pipeline_consume(uav_imu_pipeline_t *p, uav_imu_frame_t *f) {
    if (!p || !f || !p->sample.valid || !p->integrated_us) return -1;
    f->dt_s = p->integrated_us * 1e-6f;
    f->timestamp_us = p->sample.timestamp_us;
    memcpy(f->acc, p->sample.acc, sizeof(f->acc));
    memcpy(f->gyro_control, p->sample.gyro_control, sizeof(f->gyro_control));
    memcpy(f->gyro_calibration, p->sample.gyro_calibration, sizeof(f->gyro_calibration));
    for (unsigned i = 0; i < 3; i++) {
        f->gyro_average[i] = p->angle_integral[i] / f->dt_s - p->gyro_bias[i];
        p->angle_integral[i] = 0;
    }
    p->integrated_us = 0;
    return 0;
}
void uav_imu_pipeline_set_bias(uav_imu_pipeline_t *p, const float bias[3]) {
    if (!p || !bias) return;
    for (unsigned i = 0; i < 3; i++) if (!isfinite(bias[i])) return;
    memcpy(p->gyro_bias, bias, sizeof(p->gyro_bias));
    /* Filters always process absolute gyro; a new offset has no filter transient. */
    p->integrated_us = 0;
    memset(p->angle_integral, 0, sizeof(p->angle_integral));
}
int uav_imu_pipeline_set_accel_calibration(uav_imu_pipeline_t *p, const float bias[3], const float scale[3]) {
    if (!p || !bias || !scale) return -1;
    for (unsigned i=0;i<3;i++)
        if (!isfinite(bias[i]) || fabsf(bias[i])>1 || !isfinite(scale[i]) || scale[i]<.7f || scale[i]>1.3f) return -1;
    memcpy(p->accel_bias,bias,sizeof(p->accel_bias)); memcpy(p->accel_scale,scale,sizeof(p->accel_scale));
    discontinuity(p); return 0;
}
