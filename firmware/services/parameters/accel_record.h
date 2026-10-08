#ifndef UAV_ACCEL_RECORD_H
#define UAV_ACCEL_RECORD_H
#include "accel_calibration.h"
#include "param_keys.h"
void uav_accel_record_encode(uint8_t bytes[UAV_PARAM_ACCEL_BYTES], const uav_accel_calibration_t *cal);
int uav_accel_record_decode(const uint8_t bytes[UAV_PARAM_ACCEL_BYTES], uav_accel_calibration_t *cal);
#endif
