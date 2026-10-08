#include "calibration_record.h"
#include "star_protocol.h"
#include "mag_record.h"
#include "accel_record.h"
#include <math.h>
int calibration_record_valid(unsigned kind, const uint8_t *b, size_t n) {
    if (!b)
        return 0;
    if (kind == UAV_PARAM_ACCEL) {
        uav_accel_calibration_t c;
        return n == UAV_PARAM_ACCEL_BYTES && uav_accel_record_decode(b, &c);
    }
    if (kind == UAV_PARAM_MAG) {
        uav_mag_calibration_t c;
        return n == UAV_PARAM_MAG_BYTES && uav_mag_record_decode(b, &c);
    }
    if (kind == 1) {
        if (n != 72)
            return 0;
        for (unsigned i = 0; i < 18; i++) {
            float v = star_read_f32(b + 4 * i);
            if (!isfinite(v) || fabsf(v) > 100000 || (i >= 15 && (v < 0.1f || v > 10)))
                return 0;
        }
        return 1;
    }
    if (kind == 2) {
        if (n != 32)
            return 0;
        for (unsigned i = 0; i < 8; i++) {
            unsigned max = star_read_u16(b + i * 4), min = star_read_u16(b + i * 4 + 2);
            if (max > 2047 || min >= max || max - min < 100)
                return 0;
        }
        return 1;
    }
    if (kind == 3) {
        float sum = 0;
        if (n != 60)
            return 0;
        for (unsigned i = 0; i < 15; i++) {
            float v = star_read_f32(b + i * 4);
            if (!isfinite(v) || v < 0 || v > 2000)
                return 0;
            sum += v;
        }
        return sum > 0;
    }
    return 0;
}
