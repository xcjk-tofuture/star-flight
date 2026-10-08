#ifndef UAV_SBUS_RX_PORT_H
#define UAV_SBUS_RX_PORT_H
#include <stdint.h>
typedef struct {
    uint32_t errors, parity, noise, framing, overrun, dma_errors;
    uint32_t attempts, recovered, failed, last_error, last_status;
    uint8_t running, pending;
} uav_sbus_rx_stats_t;
void uav_sbus_rx_error_isr(uint32_t error);
uint8_t uav_sbus_rx_running(void);
uint8_t uav_sbus_rx_pending(void);
/* Task context. 1 restarted, 0 healthy/backoff, -1 retry failed. */
int uav_sbus_rx_service(uint32_t now_ms);
void uav_sbus_rx_stats_read(uav_sbus_rx_stats_t *out);
#endif
