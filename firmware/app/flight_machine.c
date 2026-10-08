#include "flight_machine.h"
#include "rc_control_config.h"
#include <string.h>
void flight_machine_init(flight_machine_t *m) { memset(m,0,sizeof(*m)); }
static void reset_arm(flight_machine_t *m) {
    m->gesture_active=0; m->neutral_active=0; m->arm_ready=0;
}
static void state_set(flight_machine_t *m, uint8_t state) {
    if (m->state!=state) { m->state=state; m->transitions++; }
}
static int center(uint16_t value) { return value>=UAV_RC_CENTER_LOW && value<=UAV_RC_CENTER_HIGH; }
void flight_machine_step(flight_machine_t *m, const flight_inputs_t *in) {
    uint8_t fault=0;
    unsigned mode=in->channels[4]<=UAV_RC_MODE_LOW ? 0:in->channels[4]>=UAV_RC_MODE_HIGH ? 2:1;
    int enabled=in->channels[5]>=UAV_RC_SAFETY_ON;
    int safety=in->channels[5]<=UAV_RC_SAFETY_OFF;
    int throttle_low=in->channels[2]<=UAV_RC_ARM_THROTTLE;
    int range_ok=1;
    for (unsigned i=0;i<8;i++) if (in->channels[i]<1000 || in->channels[i]>2000) range_ok=0;
    if (!m->mode_seen || mode!=m->previous_mode || enabled!=m->previous_enable) reset_arm(m);
    m->mode_seen=1; m->previous_mode=(uint8_t)mode; m->previous_enable=(uint8_t)enabled;
    /* A deliberate ground selection stops outputs immediately. A fault latch
     * additionally requires the physical safety position and low throttle. */
    if (in->connected && range_ok && mode==0 &&
        (m->state!=FM_EMERGENCY || (safety && throttle_low))) {
        state_set(m,FM_LOCKED); m->reason=0; reset_arm(m); return;
    }
    if (in->timing_fault || m->state>FM_EMERGENCY) fault|=FM_FAULT_TIMING;
    if (!in->connected || !range_ok) fault|=FM_FAULT_REMOTE;
    if (!in->attitude_valid || (uint32_t)(in->now_ms-in->attitude_ms)>100u) fault|=FM_FAULT_ATTITUDE;
    if (in->calibrating || in->storage_busy) fault|=FM_FAULT_CALIBRATION;
    if (!enabled && (m->state==FM_ARMED || m->state==FM_STABILIZE)) fault|=FM_FAULT_SWITCH;
    if (fault) {
        reset_arm(m);
        if (m->state!=FM_LOCKED) { state_set(m,FM_EMERGENCY); m->reason|=fault; }
        return;
    }
    if (m->state==FM_EMERGENCY) { reset_arm(m); return; }
    if (m->state==FM_LOCKED) {
        if (mode!=1 || !enabled || !throttle_low) { reset_arm(m); return; }
        int xy_center=center(in->channels[0]) && center(in->channels[1]);
        if (xy_center && center(in->channels[3])) {
            m->gesture_active=0;
            if (!m->neutral_active) { m->neutral_active=1; m->neutral_ms=in->now_ms; }
            if ((uint32_t)(in->now_ms-m->neutral_ms)>=UAV_RC_NEUTRAL_MS) m->arm_ready=1;
        } else if (xy_center && m->arm_ready && in->channels[3]>UAV_RC_CENTER_HIGH) {
            /* Retain readiness while the stick travels from center to the end.
             * The one-second timer only runs at the actual arming threshold. */
            if (in->channels[3]<UAV_RC_ARM_YAW) m->gesture_active=0;
            else {
                if (!m->gesture_active) { m->gesture_active=1; m->gesture_ms=in->now_ms; }
                if ((uint32_t)(in->now_ms-m->gesture_ms)>=UAV_RC_ARM_HOLD_MS) {
                    state_set(m,FM_ARMED); reset_arm(m);
                }
            }
        } else reset_arm(m);
    } else if (m->state==FM_ARMED && mode==2 && in->channels[2]<=1100u) {
        state_set(m,FM_STABILIZE);
    } else if (m->state==FM_STABILIZE && mode==1) {
        state_set(m,FM_ARMED);
    }
}
