#include "flight_machine.h"
#include "calibration_record.h"
#include "star_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static flight_inputs_t input(void) {
    flight_inputs_t i = {.connected = 1, .attitude_valid = 1};
    for (unsigned c = 0; c < 8; c++)
        i.channels[c] = 1000;
    i.channels[3] = 2000;
    return i;
}
static void tick(flight_machine_t *m, flight_inputs_t *i, uint32_t t) {
    i->now_ms = t;
    i->attitude_ms = t;
    flight_machine_step(m, i);
}
static void arm(flight_machine_t *m, flight_inputs_t *i) {
    tick(m, i, 0);
    tick(m, i, 999);
    assert(m->state == FM_LOCKED);
    tick(m, i, 1000);
    assert(m->state == FM_ARMED);
}
int main(void) {
    flight_machine_t m;
    flight_inputs_t i = input();
    flight_machine_init(&m);
    arm(&m, &i);
    tick(&m, &i, 3000);
    assert(m.state == FM_ARMED); /* held gesture cannot toggle repeatedly */
    i.channels[4] = 1500;
    i.channels[2] = 1500;
    tick(&m, &i, 3005);
    assert(m.state == FM_ARMED);
    i.channels[2] = 1000;
    tick(&m, &i, 3010);
    assert(m.state == FM_STABILIZE);
    i.connected = 0;
    tick(&m, &i, 3015);
    assert(m.state == FM_EMERGENCY && (m.reason & FM_FAULT_REMOTE));
    i.connected = 1;
    i.channels[2] = 1600;
    i.channels[4] = 1000;
    tick(&m, &i, 3020);
    assert(m.state == FM_EMERGENCY);
    i.channels[2] = 1000;
    tick(&m, &i, 3025);
    assert(m.state == FM_LOCKED);
    flight_machine_init(&m);
    i = input();
    arm(&m, &i);
    i.now_ms = 1201;
    i.attitude_ms = 1100;
    flight_machine_step(&m, &i);
    assert(m.state == FM_EMERGENCY && (m.reason & FM_FAULT_ATTITUDE));
    flight_machine_init(&m);
    i = input();
    arm(&m, &i);
    i.calibrating = 1;
    tick(&m, &i, 1005);
    assert(m.state == FM_EMERGENCY);
    flight_machine_init(&m);
    i = input();
    i.storage_busy = 1;
    tick(&m, &i, 0);
    tick(&m, &i, 2000);
    assert(m.state == FM_LOCKED);
    flight_machine_init(&m);
    i = input();
    i.channels[7] = 1500;
    tick(&m, &i, 0);
    assert(m.state == FM_EMERGENCY);
    flight_machine_init(&m);
    i = input();
    tick(&m, &i, UINT32_MAX - 500);
    tick(&m, &i, 499);
    assert(m.state == FM_ARMED);
    /* Broken one-second gesture must restart the timer. */
    flight_machine_init(&m);
    i = input();
    tick(&m, &i, 0);
    i.channels[0] = 1500;
    tick(&m, &i, 900);
    i.channels[0] = 1000;
    tick(&m, &i, 1000);
    tick(&m, &i, 1900);
    assert(m.state == FM_LOCKED);
    tick(&m, &i, 2000);
    assert(m.state == FM_ARMED);
    uint8_t b[72] = {0};
    flight_machine_init(&m);
    i = input();
    arm(&m, &i);
    i.timing_fault = 1;
    tick(&m, &i, 1005);
    assert(m.state == FM_EMERGENCY && (m.reason & FM_FAULT_TIMING));
    for (unsigned k = 15; k < 18; k++)
        star_write_f32(b + k * 4, 1);
    assert(calibration_record_valid(1, b, 72));
    star_write_f32(b + 60, 0);
    assert(!calibration_record_valid(1, b, 72));
    star_write_f32(b + 60, NAN);
    assert(!calibration_record_valid(1, b, 72));
    for (unsigned k = 0; k < 8; k++) {
        star_write_u16(b + k * 4, 1700);
        star_write_u16(b + k * 4 + 2, 350);
    }
    assert(calibration_record_valid(2, b, 32));
    star_write_u16(b + 4, 65535);
    assert(!calibration_record_valid(2, b, 32));
    for (unsigned k = 0; k < 15; k++)
        star_write_f32(b + k * 4, 0.1f);
    assert(calibration_record_valid(3, b, 60));
    star_write_f32(b + 4, INFINITY);
    assert(!calibration_record_valid(3, b, 60));
    puts("PASS flight/calibration: continuous gesture, wrap, throttle guards, fault "
         "latch/recovery, storage/calibration inhibit and invalid records");
    return 0;
}
