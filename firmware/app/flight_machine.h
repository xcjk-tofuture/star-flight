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
} flight_machine_t;
/* Called by one control owner. Arm/disarm requires a continuous 1s low-throttle
 * gesture. All fault conditions stop in the same tick; faults latch until a fresh
 * link/sensor, low throttle and physical CH5 lock position clear the fault.
 * CH5 high currently selects stabilize, never an unverified altitude controller. */
void flight_machine_init(flight_machine_t *m);
void flight_machine_step(flight_machine_t *m, const flight_inputs_t *in);
#endif
