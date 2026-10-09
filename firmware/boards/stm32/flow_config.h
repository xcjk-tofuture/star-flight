#ifndef UAV_FLOW_CONFIG_H
#define UAV_FLOW_CONFIG_H
#include "flow_measurement.h"
/* Vendor T2/T201 manuals confirm this 14-byte UPixels+TOF layout and binary
 * F5 validity. T201 has no gyro compensation. Exact board revision/mounting
 * remain unconfirmed; retain the conservative 8 m application limit.
 * Outputs are measurements, not flight-control inputs. */
static const uav_flow_config_t uav_board_flow_config={
    .profile=UAV_FLOW_UPIX,.minimum_flow_quality=255,.minimum_range_quality=50,
    .rotation_compensated=0,.range_min_m=.05f,.range_max_m=8.0f,
    .minimum_cos_tilt=.70710678f,.max_flow_rate_radps=4.0f,
    .maximum_uncompensated_rotation_radps=.15f,.velocity_tau_s=.053f,.timeout_ms=100
};
#endif
