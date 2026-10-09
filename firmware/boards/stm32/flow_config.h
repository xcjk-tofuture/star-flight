#ifndef UAV_FLOW_CONFIG_H
#define UAV_FLOW_CONFIG_H
#include "flow_measurement.h"
/* Vendor T2/T201 manuals confirm this 14-byte UPixels+TOF layout and binary
 * F5 validity. T201 has no gyro compensation. Exact board revision/mounting
 * remain unconfirmed; retain the conservative 8 m application limit.
 * Outputs are measurements, not flight-control inputs. */
static const uav_flow_config_t uav_board_flow_config={
    .profile=UAV_FLOW_UPIX,.minimum_flow_quality=255,.minimum_range_quality=50,
    .range_min_m=.05f,.range_max_m=8.0f,
    .minimum_cos_tilt=.70710678f,.max_flow_rate_radps=4.0f,
    .maximum_uncompensated_rotation_radps=.15f,.velocity_tau_s=.053f,.timeout_ms=100
};
/* Candidate image convention from the vendor's legacy demo: raw X~+gyro Y,
 * raw Y~-gyro X for that mounting. This is NOT confirmed for our board.
 * 10 ms is only a starting delay estimate, including module + UART latency.
 * Candidate curves are diagnostics until both confirmations are set. */
static const uav_flow_comp_config_t uav_board_flow_comp_config={
    .enabled=1,.mounting_confirmed=0,.delay_confirmed=0,.delay_us=10000,
    .image_from_body={{0,1,0},{-1,0,0}},.scale={1,1}
};
#endif
