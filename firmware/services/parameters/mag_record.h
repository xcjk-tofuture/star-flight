#ifndef UAV_MAG_RECORD_H
#define UAV_MAG_RECORD_H
#include "mag_calibration.h"
#include "param_keys.h"
void uav_mag_record_encode(uint8_t bytes[UAV_PARAM_MAG_BYTES], const uav_mag_calibration_t *cal);
int uav_mag_record_decode(const uint8_t bytes[UAV_PARAM_MAG_BYTES], uav_mag_calibration_t *cal);
#endif
