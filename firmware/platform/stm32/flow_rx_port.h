#ifndef UAV_FLOW_RX_PORT_H
#define UAV_FLOW_RX_PORT_H
#include <stdint.h>
typedef struct { uint32_t errors, attempts, recovered, failed, last_error; uint8_t running, pending; } uav_flow_rx_stats_t;
void uav_flow_rx_error_isr(uint32_t error);
uint8_t uav_flow_rx_running(void);
int uav_flow_rx_service(uint32_t now_ms);
void uav_flow_rx_stats_read(uav_flow_rx_stats_t *out);
#endif
