#ifndef FLIGHT_MACHINE_H
#define FLIGHT_MACHINE_H
#include <stdint.h>
enum { FM_LOCKED = 0, FM_ARMED = 1, FM_STABILIZE = 2, FM_EMERGENCY = 3 };
enum {
    FM_FAULT_NONE = 0,
    FM_FAULT_REMOTE = 1,
    FM_FAULT_ATTITUDE = 2,
    FM_FAULT_CALIBRATION = 4,
    FM_FAULT_SWITCH = 8,
    FM_FAULT_TIMING = 16
};
typedef struct {
    uint16_t channels[8];
    uint32_t now_ms, attitude_ms;
    uint8_t connected, attitude_valid, calibrating, storage_busy, timing_fault;
} flight_inputs_t;
typedef struct {
    uint8_t state, reason, gesture_active;
    uint32_t gesture_ms, transitions;
    uint32_t neutral_ms;
    uint8_t neutral_active, arm_ready, mode_seen, previous_mode, previous_enable;
} flight_machine_t;
/* CH5: low ground/lock, middle armed preparation, high stabilize.
 * CH6: low safety/kill, high operation permission; knobs never request a stop.
 * Arming requires centered sticks for 500ms, then low throttle + yaw right 1s.
 * Faults stop outputs; fresh link + CH5 ground + CH6 safety clears a latch. */
void flight_machine_init(flight_machine_t *m);
void flight_machine_step(flight_machine_t *m, const flight_inputs_t *in);
#endif
