#include "flow_gyro.h"
#include <math.h>
#include <string.h>
void uav_flow_gyro_history_push(uav_flow_gyro_history_t *h, uint32_t us, const float rate[3]) {
    if (!h || !rate) return;
    for (unsigned i=0;i<3;i++) if (!isfinite(rate[i]) || fabsf(rate[i])>8.6f) { h->count=0; h->generation++; return; }
    if (h->count) {
        uint32_t dt=us-h->point[(h->head+UAV_FLOW_GYRO_CAPACITY-1)%UAV_FLOW_GYRO_CAPACITY].us;
        if (dt<500u || dt>10000u) { h->count=0; h->generation++; }
    }
    h->point[h->head].us=us; memcpy(h->point[h->head].rate,rate,3*sizeof(float));
    h->head=(uint16_t)((h->head+1)%UAV_FLOW_GYRO_CAPACITY);
    if (h->count<UAV_FLOW_GYRO_CAPACITY) h->count++;
}
void uav_flow_gyro_history_integrate(const uav_flow_gyro_history_t *h, uint32_t end,
                                    uint32_t span, uav_flow_gyro_interval_t *o) {
    if (!o) return;
    memset(o,0,sizeof(*o)); o->span_us=span; o->reason=UAV_FLOW_GYRO_EMPTY;
    if (!h || h->count<2) return;
    if (span<1000u || span>65535u) { o->reason=UAV_FLOW_GYRO_INVALID; return; }
    uint32_t start=end-span;
    unsigned first=(h->head+UAV_FLOW_GYRO_CAPACITY-h->count)%UAV_FLOW_GYRO_CAPACITY;
    unsigned last=(h->head+UAV_FLOW_GYRO_CAPACITY-1)%UAV_FLOW_GYRO_CAPACITY;
    if ((int32_t)(start-h->point[first].us)<0) { o->reason=UAV_FLOW_GYRO_OLD; return; }
    if ((int32_t)(h->point[last].us-end)<0) { o->reason=UAV_FLOW_GYRO_FUTURE; return; }
    uint32_t covered=0;
    for (unsigned n=1;n<h->count;n++) {
        const uav_flow_gyro_point_t *a=&h->point[(first+n-1)%UAV_FLOW_GYRO_CAPACITY];
        const uav_flow_gyro_point_t *b=&h->point[(first+n)%UAV_FLOW_GYRO_CAPACITY];
        int32_t left=(int32_t)(a->us-start), right=(int32_t)(b->us-start);
        if (right<=0 || left>=(int32_t)span) continue;
        uint32_t dt=b->us-a->us;
        if (dt<500u || dt>10000u) { memset(o->delta,0,sizeof(o->delta)); o->reason=UAV_FLOW_GYRO_GAP; return; }
        int32_t lo=left<0 ? 0:left, hi=right>(int32_t)span ? (int32_t)span:right;
        if (hi<=lo) continue;
        float t0=(lo-left)/(float)dt, t1=(hi-left)/(float)dt;
        float seconds=(hi-lo)*1e-6f;
        for (unsigned i=0;i<3;i++) {
            float slope=b->rate[i]-a->rate[i];
            o->delta[i]+=(a->rate[i]+.5f*slope*(t0+t1))*seconds;
        }
        covered+=(uint32_t)(hi-lo);
    }
    o->valid=(uint8_t)(covered==span); o->reason=o->valid ? UAV_FLOW_GYRO_OK:UAV_FLOW_GYRO_GAP;
    if (!o->valid) memset(o->delta,0,sizeof(o->delta));
}
