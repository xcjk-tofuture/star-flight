#ifndef CALIBRATION_RECORD_H
#define CALIBRATION_RECORD_H
#include <stddef.h>
#include <stdint.h>
/* Schema 1: IMU=18 f32, remote=8 (max,min) u16, motor=15 f32, LE.
 * Raw legacy blobs have no version/CRC and are not trusted automatically. */
int calibration_record_valid(unsigned kind, const uint8_t *bytes, size_t length);
#endif
