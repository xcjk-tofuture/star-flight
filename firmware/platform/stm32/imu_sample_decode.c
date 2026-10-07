#include "imu_sample_decode.h"
int uav_imu_decode_bmi088(const uint8_t a[8], const uint8_t g[7], float acc[3], float gyro[3]) {
    if (!a || !g || !acc || !gyro) return -1;
    for (unsigned i=0;i<3;i++) {
        uint16_t ua=(uint16_t)a[2+i*2] | ((uint16_t)a[3+i*2]<<8);
        uint16_t ug=(uint16_t)g[1+i*2] | ((uint16_t)g[2+i*2]<<8);
        int32_t ra=ua>=32768u ? (int32_t)ua-65536 : ua;
        int32_t rg=ug>=32768u ? (int32_t)ug-65536 : ug;
        acc[i]=ra*(3.0f*9.80665f/32768.0f);
        gyro[i]=rg*(500.0f*0.01745329252f/32768.0f);
    }
    return 0;
}
