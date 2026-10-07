#include "mag_record.h"
#include "star_protocol.h"
void uav_mag_record_encode(uint8_t b[UAV_PARAM_MAG_BYTES], const uav_mag_calibration_t *c) {
    for (unsigned i=0;i<3;i++) star_write_f32(b+4*i,c->bias[i]);
    for (unsigned i=0;i<9;i++) star_write_f32(b+12+4*i,c->matrix[i]);
    star_write_f32(b+48,c->field_ut); star_write_f32(b+52,c->rms_fraction);
    star_write_u32(b+56,c->coverage); star_write_u32(b+60,c->samples);
}
int uav_mag_record_decode(const uint8_t b[UAV_PARAM_MAG_BYTES], uav_mag_calibration_t *c) {
    for (unsigned i=0;i<3;i++) c->bias[i]=star_read_f32(b+4*i);
    for (unsigned i=0;i<9;i++) c->matrix[i]=star_read_f32(b+12+4*i);
    c->field_ut=star_read_f32(b+48); c->rms_fraction=star_read_f32(b+52);
    c->coverage=star_read_u32(b+56); c->samples=star_read_u32(b+60);
    return uav_mag_calibration_valid(c);
}
