#include "biquad.h"
#include <math.h>
#include <string.h>

int uav_biquad_configure(uav_biquad_t *f, float rate, float cutoff) {
    if (!f || !isfinite(rate) || !isfinite(cutoff) || rate <= 0 || cutoff <= 0 || cutoff >= rate * .5f)
        return -1;
    float k = tanf(3.14159265358979323846f * cutoff / rate);
    float inverse = 1.0f / (1.0f + 1.4142135623730951f * k + k * k);
    memset(f, 0, sizeof(*f));
    f->b0 = k * k * inverse;
    f->b1 = 2 * f->b0;
    f->b2 = f->b0;
    f->a1 = 2 * (k * k - 1) * inverse;
    f->a2 = (1 - 1.4142135623730951f * k + k * k) * inverse;
    return 0;
}
void uav_biquad_reset(uav_biquad_t *f) { if (f) f->initialized = 0; }
int uav_biquad_apply(uav_biquad_t *f, const float input[3], float output[3]) {
    if (!f || !input || !output) return -1;
    for (unsigned i = 0; i < 3; i++)
        if (!isfinite(input[i])) { f->initialized = 0; return -1; }
    if (!f->initialized) {
        /* Prime both delay states at the sample's steady-state value. */
        for (unsigned i = 0; i < 3; i++) {
            f->z1[i] = input[i] * (1 - f->b0);
            f->z2[i] = input[i] * (f->b2 - f->a2);
        }
        f->initialized = 1;
    }
    for (unsigned i = 0; i < 3; i++) {
        float value = f->b0 * input[i] + f->z1[i];
        f->z1[i] = f->b1 * input[i] - f->a1 * value + f->z2[i];
        f->z2[i] = f->b2 * input[i] - f->a2 * value;
        if (!isfinite(value) || !isfinite(f->z1[i]) || !isfinite(f->z2[i])) {
            f->initialized = 0;
            return -1;
        }
        output[i] = value;
    }
    return 0;
}
