#include "flow_measurement.h"
#include <math.h>
#include <string.h>
void uav_flow_processor_init(uav_flow_processor_t *p) { memset(p,0,sizeof(*p)); }
static void range_reset(uav_flow_processor_t *p, float h, uint32_t ms) {
    p->height=h; p->vertical_speed=0; p->p00=.01f; p->p01=0; p->p11=1;
    p->range_initialized=1; p->candidate_count=0; p->last_range_ms=ms;
}
/* Two-state constant-velocity range filter. Quality affects measurement noise;
 * an innovation gate suppresses isolated echoes, three consistent new heights
 * permit reacquisition after a terrain step. */
static int range_update(uav_flow_processor_t *p, float h, float dt, float quality, uint32_t ms) {
    if (!p->range_initialized || (uint32_t)(ms-p->last_range_ms)>300u) { range_reset(p,h,ms); return 1; }
    float predicted=p->height+p->vertical_speed*dt;
    float q=.64f, dt2=dt*dt;
    float a=p->p00+2*dt*p->p01+dt2*p->p11+.25f*q*dt2*dt2;
    float b=p->p01+dt*p->p11+.5f*q*dt2*dt;
    float d=p->p11+q*dt2;
    float noise=.02f+.005f*h+.08f*(1-quality), variance=noise*noise;
    float residual=h-predicted, innovation=a+variance;
    if (residual*residual>9*innovation && fabsf(residual)>.10f) {
        if (p->candidate_count && fabsf(h-p->candidate_height)<.05f) p->candidate_count++;
        else { p->candidate_height=h; p->candidate_count=1; }
        if (p->candidate_count>=3) { range_reset(p,h,ms); return 1; }
        return 0;
    }
    p->candidate_count=0;
    float k0=a/innovation, k1=b/innovation;
    p->height=predicted+k0*residual; p->vertical_speed+=k1*residual;
    if (p->height<0) { p->height=0; if (p->vertical_speed<0) p->vertical_speed=0; }
    if (p->vertical_speed>5) p->vertical_speed=5;
    if (p->vertical_speed< -5) p->vertical_speed=-5;
    p->p00=fmaxf((1-k0)*a,1e-6f); p->p01=(1-k0)*b; p->p11=fmaxf(d-k1*b,1e-6f);
    p->last_range_ms=ms; return 1;
}
void uav_flow_processor_expire(uav_flow_processor_t *p, const uav_flow_config_t *c, uint32_t now) {
    if ((uint32_t)(now-p->output.flow_ms)>c->timeout_ms) {
        p->output.flow_valid=0; p->velocity_initialized=0;
        p->output.velocity_mps[0]=p->output.velocity_mps[1]=0;
        if ((uint32_t)(now-p->last_frame_ms)>c->timeout_ms) p->output.reason=UAV_FLOW_STALE;
    }
    if ((uint32_t)(now-p->output.range_ms)>300u) {
        p->output.range_valid=p->output.height_valid=0; p->range_initialized=0;
        p->output.vertical_velocity_mps=0;
    }
}
void uav_flow_processor_update(uav_flow_processor_t *p, const uav_flow_config_t *c,
                               const uav_flow_frame_t *f, const uav_flow_attitude_t *a, uint32_t now) {
    uav_flow_measurement_t *o=&p->output;
    p->last_frame_ms=f->received_ms;
    o->frames++; o->flow_valid=o->height_valid=o->range_valid=0; o->reason=UAV_FLOW_OK;
    o->flow_quality=c->profile==UAV_FLOW_FPM ? (f->byte10==0xf5 ? 255:0):f->byte10;
    o->range_quality=c->profile==UAV_FLOW_FPM ? 0:f->byte11;
    o->range_m=f->range_mm*.001f;
    if ((uint32_t)(now-f->received_ms)>300u) { o->reason=UAV_FLOW_STALE; goto reject; }
    float dt=f->integration_us>=1000u ? f->integration_us*.000001f:.02f;
    int ray_good=c->profile==UAV_FLOW_UPIX && o->range_quality>=c->minimum_range_quality &&
        o->range_quality<=100 && o->range_m>=c->range_min_m && o->range_m<=c->range_max_m;
    if (ray_good) { o->range_valid=1; o->range_ms=f->received_ms; }
    if ((uint32_t)(now-f->received_ms)>c->timeout_ms) { o->reason=UAV_FLOW_STALE; goto reject; }
    uint32_t midpoint=f->received_ms-f->integration_us/2000u;
    int32_t alignment=(int32_t)(a->sample_ms-midpoint);
    int pose_good=a->valid && (uint32_t)(now-a->sample_ms)<=100u &&
        alignment>=-50 && alignment<=50 && isfinite(a->roll) && isfinite(a->pitch) && isfinite(a->rate_x) && isfinite(a->rate_y);
    if (!pose_good) { o->reason=UAV_FLOW_ATTITUDE; goto reject; }
    float cos_tilt=cosf(a->roll)*cosf(a->pitch);
    if (cos_tilt<c->minimum_cos_tilt) { o->reason=UAV_FLOW_TILT; goto reject; }
    if (!ray_good) { o->reason=UAV_FLOW_RANGE; goto reject; }
    /* Slant range scales the camera's angular integral. AGL uses its vertical
     * projection separately; mixing these would introduce an extra tilt error. */
    float height=o->range_m*cos_tilt;
    uint32_t receipt_dt=f->received_ms-p->last_range_ms;
    float range_dt=receipt_dt && receipt_dt<=300u ? receipt_dt*.001f:dt;
    if (!range_update(p,height,range_dt,o->range_quality*.01f,f->received_ms)) {
        o->range_rejected++; o->reason=UAV_FLOW_OUTLIER; goto reject;
    }
    o->accepted_range++; o->height_valid=1; o->height_m=p->height; o->vertical_velocity_mps=p->vertical_speed;
    if (f->integration_us<1000u) { o->reason=UAV_FLOW_TIME; goto reject; }
    if (o->flow_quality<c->minimum_flow_quality) { o->reason=UAV_FLOW_QUALITY; goto reject; }
    if (!c->rotation_compensated && hypotf(a->rate_x,a->rate_y)>c->maximum_uncompensated_rotation_radps) {
        o->reason=UAV_FLOW_ROTATION; goto reject;
    }
    float rate[2]={f->integral_x*.0001f/dt,f->integral_y*.0001f/dt};
    if (hypotf(rate[0],rate[1])>c->max_flow_rate_radps) { o->reason=UAV_FLOW_OUTLIER; goto reject; }
    float velocity[2]={rate[0]*o->range_m,rate[1]*o->range_m};
    float alpha=dt/(c->velocity_tau_s+dt);
    if (!p->velocity_initialized || (uint32_t)(f->received_ms-o->flow_ms)>c->timeout_ms) {
        o->velocity_mps[0]=velocity[0]; o->velocity_mps[1]=velocity[1]; p->velocity_initialized=1;
    } else for (unsigned i=0;i<2;i++) o->velocity_mps[i]+=alpha*(velocity[i]-o->velocity_mps[i]);
    float sigma=(.05f+.35f*(1-o->flow_quality/255.0f))*o->range_m;
    o->velocity_variance=sigma*sigma;
    o->flow_ms=f->received_ms; o->flow_valid=1; o->accepted_flow++; return;
reject:
    o->flow_rejected++; o->velocity_mps[0]=o->velocity_mps[1]=0; p->velocity_initialized=0;
}
