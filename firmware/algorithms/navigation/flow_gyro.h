#ifndef UAV_FLOW_GYRO_H
#define UAV_FLOW_GYRO_H
#include <stdint.h>
#define UAV_FLOW_GYRO_CAPACITY 96u
typedef struct { uint32_t us; float rate[3]; } uav_flow_gyro_point_t;
typedef struct {
    uav_flow_gyro_point_t point[UAV_FLOW_GYRO_CAPACITY];
    uint16_t head, count;
    uint32_t generation;
} uav_flow_gyro_history_t;
typedef struct { float delta[3]; uint32_t span_us; uint8_t valid, reason; } uav_flow_gyro_interval_t;
enum { UAV_FLOW_GYRO_OK, UAV_FLOW_GYRO_EMPTY, UAV_FLOW_GYRO_OLD,
       UAV_FLOW_GYRO_FUTURE, UAV_FLOW_GYRO_GAP, UAV_FLOW_GYRO_INVALID };
/* One owner or external synchronization. SI rad/s, monotonic wrapping us.
 * Integrates linearly interpolated endpoints; never extrapolates across gaps. */
void uav_flow_gyro_history_push(uav_flow_gyro_history_t *h, uint32_t us, const float rate[3]);
void uav_flow_gyro_history_integrate(const uav_flow_gyro_history_t *h, uint32_t end_us,
                                    uint32_t span_us, uav_flow_gyro_interval_t *out);
#endif
