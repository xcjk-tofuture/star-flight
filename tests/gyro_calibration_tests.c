#include "gyro_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const uav_gyro_calibration_config_t config = {500, 9.80665f, .8f, .15f, .005f, .15f};
int main(void) {
    uav_gyro_calibration_t s;
    float g[3] = {-.02f, .01f, -.005f}, a[3] = {0, 0, 9.80665f};
    float bias[3] = {99, 99, 99};
    assert(uav_gyro_calibration_init(&s, &config) == 0);
    for (unsigned i = 0; i < 499; i++)
        assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_COLLECTING);
    assert(bias[0] == 99);
    /* Signed motion cannot be mistaken for negative bias. */
    g[0] = -.5f;
    assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_REJECTED && s.count == 0);
    g[0] = .5f;
    assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_REJECTED);
    g[0] = -.02f;
    for (unsigned i = 0; i < 500; i++)
        assert(uav_gyro_calibration_feed(&s, g, a, bias) ==
               (i == 499 ? UAV_GYRO_READY : UAV_GYRO_COLLECTING));
    for (unsigned i = 0; i < 3; i++) assert(fabsf(bias[i] - g[i]) < 1e-6f);
    /* Completion is latched; reinitialization starts a new calibration. */
    g[0] = 1;
    assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_READY);
    assert(fabsf(bias[0] + .02f) < 1e-6f);
    assert(uav_gyro_calibration_init(&s, &config) == 0);
    g[0] = NAN;
    assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_REJECTED);
    g[0] = 0; a[2] = 0;
    assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_REJECTED);
    a[2] = INFINITY;
    assert(uav_gyro_calibration_feed(&s, g, a, bias) == UAV_GYRO_REJECTED);
    a[2] = 9.80665f;
    /* Oscillation cancels in the mean but must fail the variance gate. */
    for (unsigned i = 0; i < 500; i++) {
        g[0] = i % 2 ? .02f : -.02f;
        assert(uav_gyro_calibration_feed(&s, g, a, bias) ==
               (i == 499 ? UAV_GYRO_REJECTED : UAV_GYRO_COLLECTING));
    }
    assert(!s.ready && s.count == 0);
    g[0] = 0;
    for (unsigned i = 0; i < 500; i++) {
        a[0] = i % 2 ? .3f : -.3f;
        assert(uav_gyro_calibration_feed(&s, g, a, bias) ==
               (i == 499 ? UAV_GYRO_REJECTED : UAV_GYRO_COLLECTING));
    }
    uav_gyro_calibration_config_t bad = config;
    bad.max_rate_rad_s = NAN;
    assert(uav_gyro_calibration_init(&s, &bad) == -1);
    assert(uav_gyro_calibration_init(&s, &config) == 0);
    assert(!uav_gyro_calibration_expire(&s,29999,30000));
    assert(uav_gyro_calibration_expire(&s,30000,30000));
    g[0]=0; a[0]=0; a[2]=9.80665f;
    assert(uav_gyro_calibration_feed(&s,g,a,bias)==UAV_GYRO_FAILED);
    assert(!s.ready && s.failed);
    assert(uav_gyro_calibration_init(&s,&config)==0 && !s.failed);
    puts("gyro calibration: signed motion, contiguous windows, variance, invalid inputs PASS");
    return 0;
}
