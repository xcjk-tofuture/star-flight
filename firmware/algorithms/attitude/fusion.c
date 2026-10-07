#include "fusion.h"
#include <math.h>
#include <string.h>
static float limit(float v, float low, float high) { return v < low ? low : v > high ? high : v; }
static int vector_finite(const float v[3]) {
    return v && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}
static float norm(const float v[3]) { return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
static void euler_quaternion(float r, float p, float y, float q[4]) {
    float cr=cosf(r*.5f), sr=sinf(r*.5f), cp=cosf(p*.5f), sp=sinf(p*.5f);
    float cy=cosf(y*.5f), sy=sinf(y*.5f);
    q[0]=cr*cp*cy+sr*sp*sy; q[1]=sr*cp*cy-cr*sp*sy;
    q[2]=cr*sp*cy+sr*cp*sy; q[3]=cr*cp*sy-sr*sp*cy;
}
static void rotate(const float q[4], const float v[3], float out[3]) {
    float tx=2*(q[2]*v[2]-q[3]*v[1]), ty=2*(q[3]*v[0]-q[1]*v[2]), tz=2*(q[1]*v[1]-q[2]*v[0]);
    out[0]=v[0]+q[0]*tx+q[2]*tz-q[3]*ty;
    out[1]=v[1]+q[0]*ty+q[3]*tx-q[1]*tz;
    out[2]=v[2]+q[0]*tz+q[1]*ty-q[2]*tx;
}
static void output_euler(uav_fusion_t *s) {
    const float *q=s->q;
    s->roll_deg=atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]))*57.295779513f;
    s->pitch_deg=-asinf(limit(2*(q[0]*q[2]-q[3]*q[1]),-1,1))*57.295779513f;
    s->yaw_deg=-atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))*57.295779513f;
}
int uav_fusion_init(uav_fusion_t *s, const uav_fusion_config_t *c) {
    if (!s || !c || !isfinite(c->gravity_m_s2) || c->gravity_m_s2 <= 0 ||
        !isfinite(c->accel_gain) || c->accel_gain < 0 || !isfinite(c->mag_gain) || c->mag_gain < 0 ||
        !isfinite(c->bias_gain) || c->bias_gain < 0 || !isfinite(c->bias_limit_rad_s) || c->bias_limit_rad_s < 0 ||
        !isfinite(c->mag_min_ut) || !isfinite(c->mag_max_ut) || c->mag_min_ut <= 0 || c->mag_max_ut <= c->mag_min_ut ||
        !isfinite(c->mag_relative_tolerance) || c->mag_relative_tolerance <= 0 || c->mag_relative_tolerance >= 1 ||
        !isfinite(c->mag_innovation_rad) || c->mag_innovation_rad <= 0 || !c->mag_timeout_us || !c->mag_recovery_samples)
        return -1;
    memset(s,0,sizeof(*s)); s->config=*c; s->q[0]=1;
    return uav_biquad_configure(&s->mag_filter,c->mag_sample_hz,c->mag_cutoff_hz);
}
int uav_fusion_step(uav_fusion_t *s, const float a[3], const float g[3], const float m[3],
                    uint32_t now, uint32_t mag_us, int available, float dt) {
    if (!s) return -1;
    if (!vector_finite(g) || !isfinite(norm(g)) || !isfinite(dt) || dt <= 0 || dt > .02f) {
        s->invalid_updates++; return -1;
    }
    float an=vector_finite(a) ? norm(a) : 0;
    float mn=vector_finite(m) ? norm(m) : 0;
    int mag_good=(available&UAV_FUSION_MAG_AVAILABLE) && (uint32_t)(now-mag_us)<=s->config.mag_timeout_us &&
        mn>=s->config.mag_min_ut && mn<=s->config.mag_max_ut && s->config.mag_gain>0;
    s->mag_norm_ut=mn;
    s->mag_reason=!(available&UAV_FUSION_MAG_AVAILABLE) ? UAV_MAG_MISSING :
        (uint32_t)(now-mag_us)>s->config.mag_timeout_us ? UAV_MAG_STALE : !mag_good ? UAV_MAG_RANGE : UAV_MAG_OK;
    if (mag_good && s->mag_reference_ut>0 &&
        fabsf(mn/s->mag_reference_ut-1)>s->config.mag_relative_tolerance) {
        mag_good=0; s->mag_reason=UAV_MAG_FIELD_CHANGE;
    }
    if (!s->initialized) {
        if (!isfinite(an) || fabsf(an/s->config.gravity_m_s2-1)>.2f) return -1;
        float r=atan2f(a[1],a[2]), p=atan2f(-a[0],sqrtf(a[1]*a[1]+a[2]*a[2])), y=0;
        euler_quaternion(r,p,0,s->q);
        float field[3]={0};
        if (mag_good) {
            rotate(s->q,m,field);
            if (hypotf(field[0],field[1])<mn*.05f) mag_good=0;
            else y=-atan2f(field[1],field[0]);
        }
        euler_quaternion(r,p,y,s->q);
        s->initialized=1;
        s->accel_weight=1;
        if (mag_good) {
            s->mag_reference_ut=mn; s->mag_good_samples=s->config.mag_recovery_samples;
            s->mag_weight=1; s->mag_used=1; s->last_mag_us=mag_us; s->have_mag_timestamp=1;
            rotate(s->q,m,s->mag_earth);
            float seeded[3]; uav_biquad_apply(&s->mag_filter,s->mag_earth,seeded);
            s->mag_horizontal_ut=hypotf(s->mag_earth[0],s->mag_earth[1]);
        }
        output_euler(s);
        return 0;
    }
    const float *q=s->q;
    float down[3]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[0]*q[1]+q[2]*q[3]),
                  q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]};
    float correction[3]={0};
    s->accel_weight=isfinite(an) && an>1e-6f ? limit(1-fabsf(an/s->config.gravity_m_s2-1)/.2f,0,1) : 0;
    if (s->accel_weight>0) {
        correction[0]=(a[1]*down[2]-a[2]*down[1])/an*s->config.accel_gain*s->accel_weight;
        correction[1]=(a[2]*down[0]-a[0]*down[2])/an*s->config.accel_gain*s->accel_weight;
        correction[2]=(a[0]*down[1]-a[1]*down[0])/an*s->config.accel_gain*s->accel_weight;
    } else s->accel_rejected++;
    float tilt_correction[3]; memcpy(tilt_correction,correction,sizeof(tilt_correction));
    float heading=0;
    if (mag_good) {
        float field[3]; rotate(q,m,field);
        s->mag_horizontal_ut=hypotf(field[0],field[1]);
        if (s->mag_horizontal_ut<mn*.05f) { mag_good=0; s->mag_reason=UAV_MAG_VERTICAL; }
        else if (!s->have_mag_timestamp || mag_us!=s->last_mag_us) {
            /* Filter the world-frame innovation vector only on fresh magnetic
             * samples, so real body rotation is carried by the gyroscope. */
            uav_biquad_apply(&s->mag_filter,field,s->mag_earth);
            if (s->mag_good_samples<s->config.mag_recovery_samples) s->mag_good_samples++;
            s->last_mag_us=mag_us; s->have_mag_timestamp=1;
        }
        heading=atan2f(s->mag_earth[1],s->mag_earth[0]);
        /* Gate the unsmoothed innovation too: smoothing must not conceal a
         * sudden magnetic jump while armed. */
        int large=fabsf(atan2f(field[1],field[0]))>s->config.mag_innovation_rad ||
                  fabsf(heading)>s->config.mag_innovation_rad;
        int stationary=norm(g)<.05f && s->accel_weight>.8f;
        if (large && (!(available&UAV_FUSION_ALLOW_MAG_RECOVERY) || !stationary)) {
            mag_good=0; s->mag_reason=UAV_MAG_INNOVATION; s->mag_good_samples=0;
        }
    }
    s->mag_innovation_rad=heading;
    if (!mag_good) {
        s->mag_good_samples=0; s->mag_weight=0; s->mag_used=0;
        uav_biquad_reset(&s->mag_filter);
        if (available&UAV_FUSION_MAG_AVAILABLE) s->mag_rejected++;
    } else {
        if (s->mag_good_samples>=s->config.mag_recovery_samples) {
            s->mag_weight=limit(s->mag_weight+dt,0,1); /* One-second reacquisition ramp. */
            s->mag_used=1;
            if (!s->mag_reference_ut) s->mag_reference_ut=mn;
            s->mag_reference_ut+=.01f*dt*(mn-s->mag_reference_ut);
            float yaw_correction=limit(-heading*s->config.mag_gain,-.15f,.15f)*s->mag_weight;
            for (unsigned i=0;i<3;i++) correction[i]+=down[i]*yaw_correction;
        } else s->mag_reason=UAV_MAG_RECOVERING;
    }
    /* Freeze online bias during rapid rotation or missing attitude references. */
    if (norm(g)<.175f && s->accel_weight>.5f)
        for (unsigned i=0;i<3;i++)
            s->gyro_bias[i]=limit(s->gyro_bias[i]+tilt_correction[i]*s->config.bias_gain*dt,
                                  -s->config.bias_limit_rad_s,s->config.bias_limit_rad_s);
    /* Gravity cannot observe rotation about Down. Never let magnetic noise
     * learn a yaw-rate offset that would keep spinning after mag rejection. */
    float unobservable=0;
    for (unsigned i=0;i<3;i++) unobservable+=s->gyro_bias[i]*down[i];
    for (unsigned i=0;i<3;i++) s->gyro_bias[i]-=unobservable*down[i];
    float peak=fmaxf(fabsf(s->gyro_bias[0]),fmaxf(fabsf(s->gyro_bias[1]),fabsf(s->gyro_bias[2])));
    if (peak>s->config.bias_limit_rad_s && peak>0)
        for (unsigned i=0;i<3;i++) s->gyro_bias[i]*=s->config.bias_limit_rad_s/peak;
    float angle[3],theta2=0;
    for (unsigned i=0;i<3;i++) { angle[i]=(g[i]+s->gyro_bias[i]+correction[i])*dt; theta2+=angle[i]*angle[i]; }
    float theta=sqrtf(theta2),scale=theta<1e-4f ? .5f-theta2/48 : sinf(theta*.5f)/theta;
    float dq[4]={cosf(theta*.5f),angle[0]*scale,angle[1]*scale,angle[2]*scale};
    float next[4]={q[0]*dq[0]-q[1]*dq[1]-q[2]*dq[2]-q[3]*dq[3],
                   q[0]*dq[1]+q[1]*dq[0]+q[2]*dq[3]-q[3]*dq[2],
                   q[0]*dq[2]-q[1]*dq[3]+q[2]*dq[0]+q[3]*dq[1],
                   q[0]*dq[3]+q[1]*dq[2]-q[2]*dq[1]+q[3]*dq[0]};
    float length=sqrtf(next[0]*next[0]+next[1]*next[1]+next[2]*next[2]+next[3]*next[3]);
    if (!isfinite(length) || length<1e-6f) { s->invalid_updates++; memset(s->gyro_bias,0,sizeof(s->gyro_bias)); return -1; }
    for (unsigned i=0;i<4;i++) s->q[i]=next[i]/length;
    output_euler(s);
    return 0;
}
