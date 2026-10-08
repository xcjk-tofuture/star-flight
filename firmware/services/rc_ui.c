#include "rc_ui.h"
#include "rc_control_config.h"
#include <stdlib.h>
#include <string.h>
static int8_t direction(uint16_t value, int8_t previous) {
    if (value<=1250) return -1;
    if (value>=1750) return 1;
    if (value>=1400 && value<=1600) return 0;
    return previous;
}
static void emit(rc_ui_action_t *out, unsigned *n, uint8_t kind, uint16_t value) {
    if (*n<4) out[(*n)++]=(rc_ui_action_t){kind,value};
}
static uint16_t bin(uint16_t value, uint16_t count) {
    unsigned result=(unsigned)(value-1000)*count/1001u;
    return (uint16_t)(result<count ? result:count-1u);
}
unsigned rc_ui_update(rc_ui_t *u, const rc_ui_frame_t *f, const rc_ui_context_t *c, rc_ui_action_t out[4]) {
    int gate=f->connected && (uint32_t)(c->now_ms-f->sample_ms)<=100u && c->state==0 &&
        !c->saving && f->channels[4]<=UAV_RC_MODE_LOW && f->channels[5]<=UAV_RC_SAFETY_OFF &&
        f->channels[2]<=UAV_RC_UI_THROTTLE;
    for (unsigned i=0;i<8;i++) if (f->channels[i]<1000 || f->channels[i]>2000) gate=0;
    if (!gate) { memset(u,0,sizeof(*u)); return 0; }
    if (!u->active) {
        u->active=1; u->context_id=UINT32_MAX;
        u->knob_anchor[0]=f->channels[6]; u->knob_anchor[1]=f->channels[7];
    }
    int centered=f->channels[0]>=1400 && f->channels[0]<=1600 && f->channels[1]>=1400 &&
        f->channels[1]<=1600 && f->channels[3]>=1400 && f->channels[3]<=1600;
    if (!u->ready) {
        if (!centered) { u->neutral=0; return 0; }
        if (!u->neutral) { u->neutral=1; u->neutral_ms=c->now_ms; }
        if ((uint32_t)(c->now_ms-u->neutral_ms)<200u) return 0;
        u->ready=1;
    }
    uint32_t id=((uint32_t)c->screen<<24)|((uint32_t)c->page<<16)|c->control_context;
    if (id!=u->context_id) {
        u->context_id=id;
        u->knob_anchor[0]=f->channels[6]; u->knob_anchor[1]=f->channels[7];
        u->knob_bin[0]=u->knob_candidate[0]=c->select_count ? bin(f->channels[6],c->select_count):0;
        u->knob_bin[1]=u->knob_candidate[1]=c->view_count ? bin(f->channels[7],c->view_count):0;
        u->knob_candidate_anchor[0]=f->channels[6]; u->knob_candidate_anchor[1]=f->channels[7];
        u->knob_ms[0]=u->knob_ms[1]=c->now_ms;
    }
    unsigned n=0;
    int8_t pitch=direction(f->channels[1],u->pitch),roll=direction(f->channels[0],u->roll),
           yaw=direction(f->channels[3],u->yaw);
    if (yaw!=u->yaw) { u->yaw_ms=c->now_ms; u->yaw_sent=0; }
    /* During range collection, only deliberate save/cancel holds are accepted.
     * Recalibration uses the previous valid ranges; first calibration uses board keys. */
    int collecting=c->remote_calibrating && c->remote_capture_view;
    if (collecting) {
        if (yaw && !u->yaw_sent && (uint32_t)(c->now_ms-u->yaw_ms)>=1500u) {
            if (yaw<0) { emit(out,&n,RC_UI_CAL,0); u->yaw_sent=1; }
            else if (c->remote_ranges_ready) { emit(out,&n,RC_UI_SAVE_DIALOG,0); u->yaw_sent=1; }
        }
    } else {
        if (pitch && (pitch!=u->pitch || (uint32_t)(c->now_ms-u->pitch_ms)>=350u)) {
            emit(out,&n,pitch>0 ? RC_UI_PREVIOUS:RC_UI_NEXT,0); u->pitch_ms=c->now_ms;
        }
        if (roll && roll!=u->roll) emit(out,&n,roll>0 ? RC_UI_ENTER:RC_UI_BACK,0);
        if (yaw>0 && !u->yaw_sent && (uint32_t)(c->now_ms-u->yaw_ms)>=700u) {
            emit(out,&n,RC_UI_CAL,0); u->yaw_sent=1;
        }
        if (yaw<0 && !u->yaw_sent && (uint32_t)(c->now_ms-u->yaw_ms)>=700u) {
            emit(out,&n,RC_UI_BACK,0); u->yaw_sent=1;
        }
    }
    u->pitch=pitch; u->roll=roll; u->yaw=yaw;
    for (unsigned knob=0;knob<2;knob++) {
        uint16_t count=knob ? c->view_count:(collecting ? 0:c->select_count);
        if (!count) continue;
        uint16_t value=f->channels[6+knob];
        uint16_t candidate=bin(value,count);
        /* Dense numeric ranges can have one bin per PWM unit. A small jitter
         * band prevents ADC noise from endlessly restarting the settle timer. */
        if (candidate!=u->knob_candidate[knob] && (count<=255u ||
            abs((int)value-(int)u->knob_candidate_anchor[knob])>=8)) {
            u->knob_candidate[knob]=candidate; u->knob_ms[knob]=c->now_ms;
            u->knob_candidate_anchor[knob]=value;
        }
        candidate=u->knob_candidate[knob];
        uint16_t displayed=knob ? c->view_index:c->select_index;
        if (candidate!=displayed && abs((int)value-(int)u->knob_anchor[knob])>=40 &&
            (uint32_t)(c->now_ms-u->knob_ms[knob])>=150u) {
            emit(out,&n,knob ? RC_UI_VIEW:RC_UI_SELECT,candidate);
            u->knob_bin[knob]=candidate; u->knob_anchor[knob]=value;
        }
    }
    return n;
}
