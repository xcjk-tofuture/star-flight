#ifndef UAV_FLOW_MEASUREMENT_H
#define UAV_FLOW_MEASUREMENT_H
#include "flow_stream.h"
enum { UAV_FLOW_UPIX=1, UAV_FLOW_FPM=2 };
enum { UAV_FLOW_OK=0, UAV_FLOW_STALE, UAV_FLOW_TIME, UAV_FLOW_QUALITY, UAV_FLOW_RANGE,
       UAV_FLOW_ATTITUDE, UAV_FLOW_TILT, UAV_FLOW_ROTATION, UAV_FLOW_OUTLIER };
typedef struct {
    uint8_t profile, minimum_flow_quality, minimum_range_quality, rotation_compensated;
    float range_min_m, range_max_m, minimum_cos_tilt, max_flow_rate_radps;
    float maximum_uncompensated_rotation_radps, velocity_tau_s;
    uint32_t timeout_ms;
} uav_flow_config_t;
typedef struct { uint32_t sample_ms; float roll, pitch, rate_x, rate_y; uint8_t valid; } uav_flow_attitude_t;
typedef struct {
    float range_m, height_m, vertical_velocity_mps, velocity_mps[2], velocity_variance;
    uint32_t flow_ms, range_ms, frames, accepted_flow, accepted_range, range_rejected, flow_rejected;
    uint8_t range_valid, height_valid, flow_valid, flow_quality, range_quality, reason;
} uav_flow_measurement_t;
typedef struct {
    uav_flow_measurement_t output;
    float height, vertical_speed, p00, p01, p11, candidate_height;
    uint32_t last_range_ms, last_frame_ms;
    uint8_t range_initialized, velocity_initialized, candidate_count;
} uav_flow_processor_t;
/* Sensor-axis velocities and local AGL height. No global altitude, inertial EKF,
 * motor writes or assumed body-axis mapping. Configuration must match the module. */
void uav_flow_processor_init(uav_flow_processor_t *processor);
void uav_flow_processor_update(uav_flow_processor_t *processor, const uav_flow_config_t *config,
                               const uav_flow_frame_t *frame, const uav_flow_attitude_t *attitude,
                               uint32_t now_ms);
void uav_flow_processor_expire(uav_flow_processor_t *processor, const uav_flow_config_t *config, uint32_t now_ms);
#endif
