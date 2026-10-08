#include "accel_calibration.h"
#include <math.h>
#include <string.h>
static void window_reset(uav_accel_calibrator_t *c) {
    c->count=0; c->stable=0; c->noise_latched=0;
    memset(c->mean,0,sizeof(c->mean)); memset(c->m2,0,sizeof(c->m2));
    memset(c->raw_mean,0,sizeof(c->raw_mean)); memset(c->raw_m2,0,sizeof(c->raw_m2));
}
void uav_accel_calibrator_init(uav_accel_calibrator_t *c) {
    memset(c,0,sizeof(*c)); c->detected=255; c->reason=UAV_ACCEL_REASON_PLACE;
}
void uav_accel_calibrator_confirm(uav_accel_calibrator_t *c) { (void)c; }
void uav_accel_calibrator_discard_window(uav_accel_calibrator_t *c) {
    if (c->phase>UAV_ACCEL_SAMPLE) return;
    window_reset(c); c->have_time=0;
    if (c->phase==UAV_ACCEL_SAMPLE) c->phase=UAV_ACCEL_SETTLE;
    c->reason=UAV_ACCEL_REASON_TIMING;
}
static unsigned detect(const float a[3]) {
    unsigned axis=0;
    for (unsigned i=1;i<3;i++) if (fabsf(a[i])>fabsf(a[axis])) axis=i;
    if (fabsf(a[axis])<7.5f || fabsf(a[axis])>12.5f) return 255;
    float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!isfinite(norm) || fabsf(a[axis])<.82f*norm) return 255;
    return (axis==2 ? 0u:axis==0 ? 2u:4u)+(a[axis]<0);
}
int uav_accel_calibrator_feed(uav_accel_calibrator_t *c, const float raw[3],
                             const float a[3], const float g[3], uint32_t ms) {
    if (c->phase>=UAV_ACCEL_FIT) return c->mask==63;
    int motion=0;
    float norm=0,gyro_norm=0;
    for (unsigned i=0;i<3;i++) {
        if (!isfinite(raw[i]) || !isfinite(a[i]) || !isfinite(g[i]) || fabsf(raw[i])>30) {
            motion=1; break;
        }
        norm+=raw[i]*raw[i]; gyro_norm+=g[i]*g[i];
        if (fabsf(raw[i]-a[i])>1.5f) motion=1;
    }
    if (!isfinite(norm) || norm<6.8f*6.8f || norm>13.0f*13.0f || gyro_norm>.08f*.08f) motion=1;
    c->detected=motion ? 255:(uint8_t)detect(a);
    if (c->have_time && (uint32_t)(ms-c->last_ms)<10u) return 0;
    int gap=c->have_time && (uint32_t)(ms-c->last_ms)>30u;
    c->last_ms=ms; c->have_time=1;
    if (motion || gap || c->detected>=6) {
        window_reset(c); c->phase=UAV_ACCEL_PLACE;
        c->reason=gap ? UAV_ACCEL_REASON_TIMING:motion ? UAV_ACCEL_REASON_MOTION:UAV_ACCEL_REASON_ORIENTATION;
        return 0;
    }
    if (c->mask & (1u<<c->detected)) {
        window_reset(c); c->phase=UAV_ACCEL_PLACE; c->reason=UAV_ACCEL_REASON_PLACE;
        c->target=0; while (c->target<6 && (c->mask & (1u<<c->target))) c->target++;
        return 0;
    }
    if (c->phase==UAV_ACCEL_PLACE || c->target!=c->detected) {
        window_reset(c); c->target=c->detected; c->phase=UAV_ACCEL_SETTLE;
    }
    if (c->noise_latched && (uint32_t)(ms-c->noisy_ms)<700u) {
        c->reason=UAV_ACCEL_REASON_NOISE; return 0;
    }
    c->noise_latched=0;
    if (!c->stable) { c->stable=1; c->stable_ms=ms; }
    if ((uint32_t)(ms-c->stable_ms)<800u) { c->reason=UAV_ACCEL_REASON_OK; return 0; }
    c->phase=UAV_ACCEL_SAMPLE; c->reason=UAV_ACCEL_REASON_OK; c->count++;
    for (unsigned i=0;i<3;i++) {
        float delta=a[i]-c->mean[i]; c->mean[i]+=delta/c->count;
        c->m2[i]+=delta*(a[i]-c->mean[i]);
        delta=raw[i]-c->raw_mean[i]; c->raw_mean[i]+=delta/c->count;
        c->raw_m2[i]+=delta*(raw[i]-c->raw_mean[i]);
    }
    if (c->count==UAV_ACCEL_CAL_SAMPLES/2) memcpy(c->half_faces[c->target],c->mean,sizeof(c->mean));
    if (c->count<UAV_ACCEL_CAL_SAMPLES) return 0;
    for (unsigned i=0;i<3;i++) {
        if (c->m2[i]/(c->count-1)>.12f*.12f || c->raw_m2[i]/(c->count-1)>.30f*.30f) {
            window_reset(c); c->phase=UAV_ACCEL_SETTLE; c->reason=UAV_ACCEL_REASON_NOISE;
            c->noise_latched=1; c->noisy_ms=ms; return 0;
        }
    }
    memcpy(c->faces[c->target],c->mean,sizeof(c->mean)); c->mask|=(uint8_t)(1u<<c->target);
    if (c->mask==63) { c->phase=UAV_ACCEL_FIT; return 1; }
    c->target=0; while (c->mask & (1u<<c->target)) c->target++;
    window_reset(c); c->phase=UAV_ACCEL_PLACE; c->reason=UAV_ACCEL_REASON_PLACE;
    return 0;
}
int uav_accel_calibration_valid(const uav_accel_calibration_t *c) {
    if (!c || c->faces!=63 || c->samples!=6*UAV_ACCEL_CAL_SAMPLES ||
        !isfinite(c->gravity_m_s2) || fabsf(c->gravity_m_s2-UAV_ACCEL_CAL_GRAVITY)>.01f ||
        !isfinite(c->rms_fraction) || c->rms_fraction<0 || c->rms_fraction>.02f) return 0;
    for (unsigned i=0;i<3;i++)
        if (!isfinite(c->bias[i]) || fabsf(c->bias[i])>1.0f || !isfinite(c->scale[i]) ||
            c->scale[i]<.7f || c->scale[i]>1.3f) return 0;
    return 1;
}
int uav_accel_calibrator_fit(uav_accel_calibrator_t *c, uav_accel_calibration_t *out) {
    if (c->mask!=63) return UAV_ACCEL_REASON_FIT;
    uav_accel_calibration_t result={.gravity_m_s2=UAV_ACCEL_CAL_GRAVITY,.faces=63,.samples=6*UAV_ACCEL_CAL_SAMPLES};
    /* Six actual gravity directions, no assumption that other axes are zero.
     * In g units: A*x^2+B*y^2+C*z^2+D*x+E*y+F*z=1. */
    float solve[6][7];
    for (unsigned face=0;face<6;face++) {
        float x=c->faces[face][0]/UAV_ACCEL_CAL_GRAVITY,
              y=c->faces[face][1]/UAV_ACCEL_CAL_GRAVITY,z=c->faces[face][2]/UAV_ACCEL_CAL_GRAVITY;
        float features[7]={x*x,y*y,z*z,x,y,z,1}; memcpy(solve[face],features,sizeof(features));
    }
    for (unsigned p=0;p<6;p++) {
        unsigned pivot=p;
        for (unsigned i=p+1;i<6;i++) if (fabsf(solve[i][p])>fabsf(solve[pivot][p])) pivot=i;
        if (fabsf(solve[pivot][p])<1e-5f) return UAV_ACCEL_REASON_FIT;
        for (unsigned j=p;j<7;j++) { float t=solve[p][j]; solve[p][j]=solve[pivot][j]; solve[pivot][j]=t; }
        float divisor=solve[p][p]; for (unsigned j=p;j<7;j++) solve[p][j]/=divisor;
        for (unsigned i=0;i<6;i++) if (i!=p) {
            float factor=solve[i][p]; for (unsigned j=p;j<7;j++) solve[i][j]-=factor*solve[p][j];
        }
    }
    float center[3],radius=1;
    for (unsigned i=0;i<3;i++) {
        if (!isfinite(solve[i][6]) || solve[i][6]<=0) return UAV_ACCEL_REASON_FIT;
        center[i]=-.5f*solve[i+3][6]/solve[i][6];
        radius+=solve[i][6]*center[i]*center[i];
        result.bias[i]=center[i]*UAV_ACCEL_CAL_GRAVITY;
    }
    for (unsigned i=0;i<3;i++) result.scale[i]=sqrtf(solve[i][6]/radius);
    float squared=0,worst=0; unsigned worst_face=0;
    for (unsigned face=0;face<6;face++) {
        for (unsigned half=0;half<2;half++) {
            float norm=0;
            for (unsigned axis=0;axis<3;axis++) {
                float value=half ? 2*c->faces[face][axis]-c->half_faces[face][axis]:c->half_faces[face][axis];
                float corrected=(value-result.bias[axis])*result.scale[axis]; norm+=corrected*corrected;
            }
            float residual=sqrtf(norm)/UAV_ACCEL_CAL_GRAVITY-1; squared+=residual*residual;
            if (residual*residual>worst) { worst=residual*residual; worst_face=face; }
        }
    }
    result.rms_fraction=sqrtf(squared/12); *out=result;
    if (worst>.02f*.02f) {
        c->mask&=(uint8_t)~(1u<<worst_face); c->target=(uint8_t)worst_face;
        window_reset(c); c->phase=UAV_ACCEL_PLACE; c->reason=UAV_ACCEL_REASON_FIT;
        return UAV_ACCEL_REASON_FIT;
    }
    return uav_accel_calibration_valid(&result) ? UAV_ACCEL_REASON_OK:UAV_ACCEL_REASON_FIT;
}
