#ifndef UAV_IMU_SAMPLE_DECODE_H
#define UAV_IMU_SAMPLE_DECODE_H
#include <stdint.h>
/* Coherent SPI replies include command/dummy bytes: ACC8, GYRO7. */
int uav_imu_decode_bmi088(const uint8_t acc_reply[8], const uint8_t gyro_reply[7],
                         float acc[3], float gyro[3]);
#endif
