#ifndef UAV_IMU_HEATER_H
#define UAV_IMU_HEATER_H
#include <stdint.h>
enum { UAV_HEATER_OFF=0, UAV_HEATER_WAIT, UAV_HEATER_WARMING, UAV_HEATER_READY, UAV_HEATER_FAULT };
enum { UAV_HEATER_OK=0, UAV_HEATER_SENSOR, UAV_HEATER_OVERHEAT, UAV_HEATER_NO_RISE, UAV_HEATER_TIMEOUT };
typedef struct {
    float temperature, target, duty_percent, p, i, d;
    uint32_t sample_ms;
    uint8_t state, fault, temperature_valid;
} uav_heater_stats_t;
void uav_imu_heater_init(void);
/* Sensor task owns samples/control. Key task provides an independent stale-data stop. */
void uav_imu_heater_sample(float temperature, uint8_t valid, uint32_t now_ms);
void uav_imu_heater_tick(uint32_t now_ms);
void uav_imu_heater_watchdog(uint32_t now_ms);
void uav_imu_heater_pause(void);
void uav_imu_heater_stats(uav_heater_stats_t *out);
#endif
