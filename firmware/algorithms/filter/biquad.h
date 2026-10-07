#ifndef UAV_BIQUAD_H
#define UAV_BIQUAD_H
#include <stdint.h>
/* Three-axis second-order Butterworth low-pass, unity DC gain. */
typedef struct {
    float b0, b1, b2, a1, a2;
    float z1[3], z2[3];
    uint8_t initialized;
} uav_biquad_t;
int uav_biquad_configure(uav_biquad_t *filter, float sample_hz, float cutoff_hz);
void uav_biquad_reset(uav_biquad_t *filter);
int uav_biquad_apply(uav_biquad_t *filter, const float input[3], float output[3]);
#endif
