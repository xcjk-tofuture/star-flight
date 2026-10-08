#include "accel_record.h"
#include "star_protocol.h"
void uav_accel_record_encode(uint8_t b[UAV_PARAM_ACCEL_BYTES], const uav_accel_calibration_t *c) {
    for (unsigned i=0;i<3;i++) { star_write_f32(b+4*i,c->bias[i]); star_write_f32(b+12+4*i,c->scale[i]); }
    star_write_f32(b+24,c->gravity_m_s2); star_write_f32(b+28,c->rms_fraction);
    star_write_u32(b+32,c->faces); star_write_u32(b+36,c->samples);
}
int uav_accel_record_decode(const uint8_t b[UAV_PARAM_ACCEL_BYTES], uav_accel_calibration_t *c) {
    for (unsigned i=0;i<3;i++) { c->bias[i]=star_read_f32(b+4*i); c->scale[i]=star_read_f32(b+12+4*i); }
    c->gravity_m_s2=star_read_f32(b+24); c->rms_fraction=star_read_f32(b+28);
    c->faces=star_read_u32(b+32); c->samples=star_read_u32(b+36);
    return uav_accel_calibration_valid(c);
}
