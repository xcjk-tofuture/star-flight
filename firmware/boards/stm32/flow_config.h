#ifndef UAV_FLOW_CONFIG_H
#define UAV_FLOW_CONFIG_H
#include "flow_measurement.h"
/* Historical 977d5b0 names the module upixels_flow_T2_data. Use its combined
 * UPix layout as the compatibility baseline; mounting/gyro compensation are
 * still unconfirmed. Outputs are measurements, not flight-control inputs. */
static const uav_flow_config_t uav_board_flow_config={
    .profile=UAV_FLOW_UPIX,.minimum_flow_quality=40,.minimum_range_quality=50,
    .rotation_compensated=0,.range_min_m=.05f,.range_max_m=8.0f,
    .minimum_cos_tilt=.70710678f,.max_flow_rate_radps=4.0f,
    .maximum_uncompensated_rotation_radps=.15f,.velocity_tau_s=.053f,.timeout_ms=100
};
#endif
