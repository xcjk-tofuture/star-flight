#ifndef UAV_FLOW_GYRO_SERVICE_H
#define UAV_FLOW_GYRO_SERVICE_H
#include "flow_gyro.h"
/* Sensor task publishes unfiltered, bias-corrected body gyro at 500 Hz.
 * Flow task is the only reader. Static storage, no stack-sized history copy. */
void uav_flow_gyro_publish(uint32_t us, const float rate[3], uint8_t valid);
void uav_flow_gyro_read(uint32_t end_us, uint32_t span_us, uav_flow_gyro_interval_t *out);
#endif
