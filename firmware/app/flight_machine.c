#include "flight_machine.h"
#include <string.h>
void flight_machine_init(flight_machine_t *m) { memset(m, 0, sizeof(*m)); }
void flight_machine_step(flight_machine_t *m, const flight_inputs_t *in) {
    uint8_t before = m->state, fault = 0;
    if (in->timing_fault || m->state > FM_EMERGENCY)
        fault |= FM_FAULT_TIMING;
    if (!in->connected)
        fault |= FM_FAULT_REMOTE;
    if (!in->attitude_valid || (uint32_t)(in->now_ms - in->attitude_ms) > 100)
        fault |= FM_FAULT_ATTITUDE;
    if (in->calibrating || in->storage_busy)
        fault |= FM_FAULT_CALIBRATION;
    for (unsigned i = 0; i < 8; i++)
        if (in->channels[i] < 1000 || in->channels[i] > 2000)
            fault |= FM_FAULT_REMOTE;
    if (in->channels[7] > 1400)
        fault |= FM_FAULT_SWITCH;
    if (fault) {
        m->gesture_active = 0;
        if (m->state != FM_LOCKED || (fault & FM_FAULT_SWITCH)) {
            m->state = FM_EMERGENCY;
            m->reason |= fault;
        }
    } else if (m->state == FM_EMERGENCY) {
        m->gesture_active = 0;
        if (in->channels[2] <= 1050 && in->channels[4] <= 1100) {
            m->state = FM_LOCKED;
            m->reason = 0;
        }
    } else {
        uint8_t gesture = in->channels[0] <= 1050 && in->channels[1] <= 1050 &&
                          in->channels[2] <= 1050 && in->channels[3] >= 1950 &&
                          in->channels[4] <= 1100;
        if (gesture && m->state != FM_STABILIZE) {
            if (!m->gesture_active) {
                m->gesture_active = 1;
                m->gesture_ms = in->now_ms;
            } else if ((uint32_t)(in->now_ms - m->gesture_ms) >= 1000 && m->gesture_active == 1) {
                m->state = m->state == FM_LOCKED ? FM_ARMED : FM_LOCKED;
                m->gesture_active = 2;
            }
        } else
            m->gesture_active = 0;
        if (m->state == FM_ARMED && in->channels[4] > 1400 && in->channels[2] <= 1100)
            m->state = FM_STABILIZE;
        else if (m->state == FM_STABILIZE && in->channels[4] <= 1100)
            m->state = FM_ARMED;
    }
    if (before != m->state)
        m->transitions++;
}
